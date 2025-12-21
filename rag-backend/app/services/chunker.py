"""
Semantic chunker service for splitting book content into chunks.

Handles:
- Boundary-aware chunking (paragraphs, code blocks, lists, tables)
- Token counting with tiktoken
- Configurable chunk sizes with overlap
- Content type preservation
"""
import re
import os
from typing import List, Tuple
import tiktoken

from app.models.book_content import BookContent
from app.models.chunk import Chunk
from app.services.content_loader import ContentLoader


class SemanticChunker:
    """Chunks book content at semantic boundaries while respecting token limits."""

    def __init__(
        self,
        target_size: int = 400,
        min_size: int = 100,
        max_size: int = 700,
        overlap_percent: float = 0.15,
    ):
        """
        Initialize semantic chunker.

        Args:
            target_size: Target chunk size in tokens (default: 400)
            min_size: Minimum chunk size in tokens (default: 100)
            max_size: Maximum chunk size in tokens (default: 700)
            overlap_percent: Overlap percentage between chunks (default: 0.15 for 15%)
        """
        self.target_size = target_size
        self.min_size = min_size
        self.max_size = max_size
        self.overlap_percent = overlap_percent

        # Initialize tiktoken encoder (cl100k_base is used by GPT-4 and similar models)
        self.encoder = tiktoken.get_encoding("cl100k_base")

        # Initialize content loader for deep link generation
        base_url = os.getenv("DOCUSAURUS_BASE_URL", "http://localhost:3000")
        self.content_loader = ContentLoader(base_url=base_url)

    def semantic_chunk(self, content: BookContent) -> List[Chunk]:
        """
        Chunk content at semantic boundaries while respecting token limits.

        Args:
            content: BookContent object to chunk

        Returns:
            List of Chunk objects
        """
        # Special handling for certain content types - keep intact
        if content.content_type in ["code", "glossary", "reference", "figure_caption"]:
            return self._chunk_as_single_unit(content)

        # Split content into semantic units (paragraphs, code blocks, lists, tables)
        semantic_units = self._detect_boundaries(content.content)

        # Create chunks respecting boundaries and token limits
        chunks = self._create_chunks(
            semantic_units,
            content.file_path,
            content.chapter,
            content.section,
            content.content_type,
        )

        return chunks

    def _chunk_as_single_unit(self, content: BookContent) -> List[Chunk]:
        """
        Create a single chunk for content that should not be split.

        Args:
            content: BookContent object

        Returns:
            List with single Chunk
        """
        token_count = self.count_tokens(content.content)

        # Generate paragraph ID and deep link
        paragraph_id = content.paragraph_id or self.content_loader._generate_paragraph_id(
            content.chapter, content.section, 1
        )
        deep_link_url = self.content_loader.generate_deep_link_url(
            content.file_path, paragraph_id
        )

        chunk = Chunk(
            text_content=content.content,
            chapter=content.chapter,
            section=content.section or "",
            subsection=None,
            content_type=content.content_type,
            paragraph_id=paragraph_id,
            deep_link_url=deep_link_url,
            token_count=token_count,
            overlap_start=0,
            overlap_end=0,
        )

        return [chunk]

    def _detect_boundaries(self, text: str) -> List[Tuple[str, str]]:
        """
        Detect semantic boundaries in text (paragraphs, code blocks, lists, tables).

        Args:
            text: Text content to split

        Returns:
            List of (unit_type, content) tuples
        """
        units = []
        current_unit = []
        current_type = "paragraph"
        in_code_block = False
        in_list = False
        in_table = False

        lines = text.split("\n")

        for i, line in enumerate(lines):
            # Code block detection
            if line.strip().startswith("```"):
                if in_code_block:
                    # End of code block
                    current_unit.append(line)
                    units.append(("code", "\n".join(current_unit)))
                    current_unit = []
                    current_type = "paragraph"
                    in_code_block = False
                else:
                    # Start of code block
                    if current_unit:
                        units.append((current_type, "\n".join(current_unit)))
                        current_unit = []
                    current_type = "code"
                    current_unit.append(line)
                    in_code_block = True
                continue

            # If in code block, keep everything together
            if in_code_block:
                current_unit.append(line)
                continue

            # List detection (- , * , 1. , etc.)
            is_list_item = bool(re.match(r"^\s*[-*+]\s+", line) or re.match(r"^\s*\d+\.\s+", line))

            if is_list_item:
                if not in_list:
                    # Start of list
                    if current_unit:
                        units.append((current_type, "\n".join(current_unit)))
                        current_unit = []
                    current_type = "list"
                    in_list = True
                current_unit.append(line)
            elif in_list and line.strip() and not is_list_item:
                # End of list
                units.append((current_type, "\n".join(current_unit)))
                current_unit = [line]
                current_type = "paragraph"
                in_list = False
            elif in_list and not line.strip():
                # Empty line in list - might be end
                current_unit.append(line)
            else:
                # Regular paragraph handling
                if not line.strip():
                    # Empty line - paragraph boundary
                    if current_unit:
                        units.append((current_type, "\n".join(current_unit)))
                        current_unit = []
                        current_type = "paragraph"
                else:
                    current_unit.append(line)

        # Add remaining unit
        if current_unit:
            units.append((current_type, "\n".join(current_unit)))

        return [(t, c) for t, c in units if c.strip()]

    def _create_chunks(
        self,
        semantic_units: List[Tuple[str, str]],
        file_path: str,
        chapter: str,
        section: str,
        content_type: str,
    ) -> List[Chunk]:
        """
        Create chunks from semantic units respecting token limits.

        Args:
            semantic_units: List of (unit_type, content) tuples
            file_path: File path for deep links
            chapter: Chapter title
            section: Section title
            content_type: Content type

        Returns:
            List of Chunk objects
        """
        chunks = []
        current_chunk_content = []
        current_chunk_tokens = 0
        chunk_index = 1

        for unit_type, unit_content in semantic_units:
            unit_tokens = self.count_tokens(unit_content)

            # If unit is too large, split it
            if unit_tokens > self.max_size:
                # Flush current chunk if not empty
                if current_chunk_content:
                    chunks.append(
                        self._create_chunk(
                            "\n\n".join(current_chunk_content),
                            file_path,
                            chapter,
                            section,
                            content_type,
                            chunk_index,
                        )
                    )
                    chunk_index += 1
                    current_chunk_content = []
                    current_chunk_tokens = 0

                # Split large unit at sentence boundaries
                split_units = self._split_large_unit(unit_content, unit_type)
                for split_content in split_units:
                    chunks.append(
                        self._create_chunk(
                            split_content,
                            file_path,
                            chapter,
                            section,
                            content_type,
                            chunk_index,
                        )
                    )
                    chunk_index += 1
            # If adding this unit would exceed max_size, start new chunk
            elif current_chunk_tokens + unit_tokens > self.max_size:
                chunks.append(
                    self._create_chunk(
                        "\n\n".join(current_chunk_content),
                        file_path,
                        chapter,
                        section,
                        content_type,
                        chunk_index,
                    )
                )
                chunk_index += 1
                current_chunk_content = [unit_content]
                current_chunk_tokens = unit_tokens
            # Add unit to current chunk
            else:
                current_chunk_content.append(unit_content)
                current_chunk_tokens += unit_tokens

            # If we've reached target size and have compatible content, finalize chunk
            if current_chunk_tokens >= self.target_size:
                chunks.append(
                    self._create_chunk(
                        "\n\n".join(current_chunk_content),
                        file_path,
                        chapter,
                        section,
                        content_type,
                        chunk_index,
                    )
                )
                chunk_index += 1
                current_chunk_content = []
                current_chunk_tokens = 0

        # Add remaining content as final chunk
        if current_chunk_content:
            final_content = "\n\n".join(current_chunk_content)
            final_tokens = self.count_tokens(final_content)

            # Merge with previous chunk if too small and compatible
            if chunks and final_tokens < self.min_size:
                prev_chunk = chunks[-1]
                merged_content = prev_chunk.text_content + "\n\n" + final_content
                merged_tokens = self.count_tokens(merged_content)

                if merged_tokens <= self.max_size:
                    # Update previous chunk
                    chunks[-1] = Chunk(
                        chunk_id=prev_chunk.chunk_id,
                        text_content=merged_content,
                        chapter=prev_chunk.chapter,
                        section=prev_chunk.section,
                        subsection=prev_chunk.subsection,
                        content_type=prev_chunk.content_type,
                        paragraph_id=prev_chunk.paragraph_id,
                        deep_link_url=prev_chunk.deep_link_url,
                        token_count=merged_tokens,
                        overlap_start=prev_chunk.overlap_start,
                        overlap_end=prev_chunk.overlap_end,
                    )
                else:
                    # Create as separate chunk even though small
                    chunks.append(
                        self._create_chunk(
                            final_content, file_path, chapter, section, content_type, chunk_index
                        )
                    )
            else:
                chunks.append(
                    self._create_chunk(
                        final_content, file_path, chapter, section, content_type, chunk_index
                    )
                )

        # Calculate overlaps between chunks
        chunks = self._calculate_overlaps(chunks)

        return chunks

    def _split_large_unit(self, content: str, unit_type: str) -> List[str]:
        """
        Split a large semantic unit at sentence boundaries.

        Args:
            content: Content to split
            unit_type: Type of unit (paragraph, list, etc.)

        Returns:
            List of split content pieces
        """
        # For code blocks, split at line breaks if needed
        if unit_type == "code":
            lines = content.split("\n")
            splits = []
            current_split = []
            current_tokens = 0

            for line in lines:
                line_tokens = self.count_tokens(line)
                if current_tokens + line_tokens > self.max_size and current_split:
                    splits.append("\n".join(current_split))
                    current_split = [line]
                    current_tokens = line_tokens
                else:
                    current_split.append(line)
                    current_tokens += line_tokens

            if current_split:
                splits.append("\n".join(current_split))

            return splits

        # For text, split at sentence boundaries
        sentences = re.split(r"(?<=[.!?])\s+", content)
        splits = []
        current_split = []
        current_tokens = 0

        for sentence in sentences:
            sentence_tokens = self.count_tokens(sentence)
            if current_tokens + sentence_tokens > self.max_size and current_split:
                splits.append(" ".join(current_split))
                current_split = [sentence]
                current_tokens = sentence_tokens
            else:
                current_split.append(sentence)
                current_tokens += sentence_tokens

        if current_split:
            splits.append(" ".join(current_split))

        return splits

    def _create_chunk(
        self,
        content: str,
        file_path: str,
        chapter: str,
        section: str,
        content_type: str,
        index: int,
    ) -> Chunk:
        """
        Create a Chunk object from content.

        Args:
            content: Chunk content
            file_path: File path for deep links
            chapter: Chapter title
            section: Section title
            content_type: Content type
            index: Chunk index

        Returns:
            Chunk object
        """
        token_count = self.count_tokens(content)
        paragraph_id = self.content_loader._generate_paragraph_id(chapter, section, index)
        deep_link_url = self.content_loader.generate_deep_link_url(file_path, paragraph_id)

        return Chunk(
            text_content=content,
            chapter=chapter,
            section=section or "",
            subsection=None,
            content_type=content_type,
            paragraph_id=paragraph_id,
            deep_link_url=deep_link_url,
            token_count=token_count,
            overlap_start=0,
            overlap_end=0,
        )

    def _calculate_overlaps(self, chunks: List[Chunk]) -> List[Chunk]:
        """
        Calculate and set overlap between adjacent chunks (10-20% overlap).

        Args:
            chunks: List of chunks

        Returns:
            List of chunks with overlaps calculated
        """
        if len(chunks) <= 1:
            return chunks

        updated_chunks = []

        for i, chunk in enumerate(chunks):
            overlap_start = 0
            overlap_end = 0

            # Calculate overlap with previous chunk
            if i > 0:
                prev_chunk = chunks[i - 1]
                overlap_tokens = int(prev_chunk.token_count * self.overlap_percent)
                overlap_start = min(overlap_tokens, prev_chunk.token_count)

            # Calculate overlap with next chunk
            if i < len(chunks) - 1:
                next_chunk = chunks[i + 1]
                overlap_tokens = int(chunk.token_count * self.overlap_percent)
                overlap_end = min(overlap_tokens, chunk.token_count)

            # Create updated chunk with overlap values
            updated_chunk = Chunk(
                chunk_id=chunk.chunk_id,
                text_content=chunk.text_content,
                chapter=chunk.chapter,
                section=chunk.section,
                subsection=chunk.subsection,
                content_type=chunk.content_type,
                paragraph_id=chunk.paragraph_id,
                deep_link_url=chunk.deep_link_url,
                token_count=chunk.token_count,
                overlap_start=overlap_start,
                overlap_end=overlap_end,
                book_version=chunk.book_version,
            )

            updated_chunks.append(updated_chunk)

        return updated_chunks

    def count_tokens(self, text: str) -> int:
        """
        Count tokens in text using tiktoken.

        Args:
            text: Text to count tokens for

        Returns:
            Number of tokens
        """
        return len(self.encoder.encode(text))


def semantic_chunk(
    content: BookContent,
    target_size: int = 400,
    min_size: int = 100,
    max_size: int = 700,
    overlap: float = 0.15,
) -> List[Chunk]:
    """
    Convenience function to chunk content.

    Args:
        content: BookContent object to chunk
        target_size: Target chunk size in tokens (default: 400)
        min_size: Minimum chunk size in tokens (default: 100)
        max_size: Maximum chunk size in tokens (default: 700)
        overlap: Overlap percentage (default: 0.15 for 15%)

    Returns:
        List of Chunk objects
    """
    chunker = SemanticChunker(
        target_size=target_size,
        min_size=min_size,
        max_size=max_size,
        overlap_percent=overlap,
    )
    return chunker.semantic_chunk(content)
