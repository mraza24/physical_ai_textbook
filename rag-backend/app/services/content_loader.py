"""
Content loader service for loading and parsing book content from Docusaurus.

Handles:
- Traversing Docusaurus docs/ directory
- Parsing markdown/MDX files with frontmatter
- Extracting hierarchy (chapter, section, subsection)
- Identifying content types
- Generating paragraph IDs and deep links
"""
import os
import re
from pathlib import Path
from typing import List, Dict, Any, Optional
import frontmatter

from app.models.book_content import BookContent


class ContentLoader:
    """Loads and parses book content from Docusaurus documentation."""

    def __init__(self, base_url: str = "http://localhost:3000"):
        """
        Initialize content loader.

        Args:
            base_url: Base URL for the Docusaurus site (for deep links)
        """
        self.base_url = base_url.rstrip("/")
        self.paragraph_counter = {}  # Track paragraph counts per section

    def load_book_files(self, book_path: str) -> List[BookContent]:
        """
        Load all book files from the Docusaurus docs/ directory.

        Traverses the directory structure, parses markdown/MDX files,
        and extracts content with metadata.

        Args:
            book_path: Path to the Docusaurus docs/ directory

        Returns:
            List of BookContent objects
        """
        book_path = Path(book_path)

        if not book_path.exists() or not book_path.is_dir():
            raise ValueError(f"Book path does not exist or is not a directory: {book_path}")

        contents = []

        # Traverse all markdown and MDX files
        for file_path in book_path.rglob("*.md"):
            if file_path.name.startswith("_"):
                # Skip private/template files
                continue

            try:
                content_items = self._parse_file(file_path, book_path)
                contents.extend(content_items)
            except Exception as e:
                print(f"Warning: Failed to parse {file_path}: {e}")
                continue

        # Also process .mdx files
        for file_path in book_path.rglob("*.mdx"):
            if file_path.name.startswith("_"):
                continue

            try:
                content_items = self._parse_file(file_path, book_path)
                contents.extend(content_items)
            except Exception as e:
                print(f"Warning: Failed to parse {file_path}: {e}")
                continue

        return contents

    def _parse_file(self, file_path: Path, base_path: Path) -> List[BookContent]:
        """
        Parse a single markdown/MDX file and extract content with metadata.

        Args:
            file_path: Path to the file
            base_path: Base docs/ directory path

        Returns:
            List of BookContent objects from this file
        """
        # Read file with frontmatter parsing
        with open(file_path, "r", encoding="utf-8") as f:
            post = frontmatter.load(f)

        # Get relative file path from docs/
        relative_path = file_path.relative_to(base_path)

        # Extract hierarchy from directory structure and headings
        chapter = self._extract_chapter(relative_path, post.metadata)
        section = self._extract_section(post.content, post.metadata)

        # Identify content type
        content_type = self._identify_content_type(file_path, post.content, post.metadata)

        # Split content into paragraphs and create BookContent objects
        contents = []

        # For code-heavy files or glossaries, treat as single content
        if content_type in ["code", "glossary", "reference"]:
            paragraph_id = self._generate_paragraph_id(chapter, section, 1)
            contents.append(
                BookContent(
                    file_path=str(relative_path),
                    content=post.content,
                    content_type=content_type,
                    chapter=chapter,
                    section=section,
                    paragraph_id=paragraph_id,
                )
            )
        else:
            # Split into paragraphs for regular text content
            paragraphs = self._split_into_paragraphs(post.content)

            for idx, paragraph_content in enumerate(paragraphs, start=1):
                if not paragraph_content.strip():
                    continue

                paragraph_id = self._generate_paragraph_id(chapter, section, idx)

                # Check if this paragraph is an exercise
                para_content_type = content_type
                if self._is_exercise(paragraph_content):
                    para_content_type = "exercise"
                elif self._is_code_block(paragraph_content):
                    para_content_type = "code"
                elif self._is_figure_caption(paragraph_content):
                    para_content_type = "figure_caption"

                contents.append(
                    BookContent(
                        file_path=str(relative_path),
                        content=paragraph_content,
                        content_type=para_content_type,
                        chapter=chapter,
                        section=section,
                        paragraph_id=paragraph_id,
                    )
                )

        return contents

    def _extract_chapter(self, relative_path: Path, metadata: Dict[str, Any]) -> str:
        """
        Extract chapter from directory structure or frontmatter.

        Args:
            relative_path: Relative file path from docs/
            metadata: Frontmatter metadata

        Returns:
            Chapter title or identifier
        """
        # Try frontmatter first
        if "chapter" in metadata:
            return metadata["chapter"]

        # Extract from directory structure (first directory level)
        parts = relative_path.parts
        if len(parts) > 1:
            # Use first directory as chapter (e.g., "module1" or "chapter2")
            chapter_dir = parts[0]
            # Clean up: "module1" -> "Module 1", "chapter-2" -> "Chapter 2"
            chapter = chapter_dir.replace("-", " ").replace("_", " ").title()
            return chapter
        else:
            # File at root level
            return "Introduction"

    def _extract_section(self, content: str, metadata: Dict[str, Any]) -> Optional[str]:
        """
        Extract section from first heading or frontmatter.

        Args:
            content: File content
            metadata: Frontmatter metadata

        Returns:
            Section title or None
        """
        # Try frontmatter first
        if "section" in metadata:
            return metadata["section"]

        # Extract first H1 or H2 heading from content
        heading_match = re.search(r"^#+\s+(.+)$", content, re.MULTILINE)
        if heading_match:
            return heading_match.group(1).strip()

        return None

    def _identify_content_type(
        self, file_path: Path, content: str, metadata: Dict[str, Any]
    ) -> str:
        """
        Identify content type based on file path, content, and metadata.

        Args:
            file_path: Path to the file
            content: File content
            metadata: Frontmatter metadata

        Returns:
            Content type: text|glossary|code|reference|figure_caption|exercise
        """
        # Check frontmatter
        if "content_type" in metadata:
            return metadata["content_type"]

        # Check file name patterns
        file_name_lower = file_path.name.lower()

        if "glossary" in file_name_lower:
            return "glossary"
        elif "reference" in file_name_lower or "bibliography" in file_name_lower:
            return "reference"

        # Check content patterns
        # If mostly code blocks, mark as code
        code_blocks = re.findall(r"```[\s\S]*?```", content)
        if code_blocks and len("".join(code_blocks)) > len(content) * 0.5:
            return "code"

        # Default to text
        return "text"

    def _split_into_paragraphs(self, content: str) -> List[str]:
        """
        Split content into paragraphs while preserving structure.

        Keeps code blocks, lists, and tables intact.

        Args:
            content: File content

        Returns:
            List of paragraph strings
        """
        # Split on double newlines, but preserve code blocks and special structures
        paragraphs = []
        current_paragraph = []
        in_code_block = False

        lines = content.split("\n")

        for line in lines:
            # Check for code block fence
            if line.strip().startswith("```"):
                in_code_block = not in_code_block
                current_paragraph.append(line)
                continue

            # If in code block, keep everything together
            if in_code_block:
                current_paragraph.append(line)
                continue

            # Empty line - potential paragraph break
            if not line.strip():
                if current_paragraph:
                    paragraphs.append("\n".join(current_paragraph))
                    current_paragraph = []
            else:
                current_paragraph.append(line)

        # Add remaining paragraph
        if current_paragraph:
            paragraphs.append("\n".join(current_paragraph))

        return [p for p in paragraphs if p.strip()]

    def _is_exercise(self, content: str) -> bool:
        """Check if content is an exercise."""
        content_lower = content.lower()
        exercise_markers = ["exercise:", "exercise ", "problem:", "practice:"]
        return any(marker in content_lower for marker in exercise_markers)

    def _is_code_block(self, content: str) -> bool:
        """Check if content is primarily a code block."""
        return content.strip().startswith("```") and content.strip().endswith("```")

    def _is_figure_caption(self, content: str) -> bool:
        """Check if content is a figure caption."""
        # Markdown image syntax: ![alt text](url)
        return bool(re.match(r"!\[.*?\]\(.*?\)", content.strip()))

    def _generate_paragraph_id(self, chapter: str, section: Optional[str], index: int) -> str:
        """
        Generate paragraph ID in format 'chapter-section-paragraph-N'.

        Args:
            chapter: Chapter title
            section: Section title
            index: Paragraph index

        Returns:
            Paragraph ID string
        """
        # Create slug from chapter
        chapter_slug = self._slugify(chapter)

        # Create slug from section if available
        section_slug = self._slugify(section) if section else "intro"

        # Format: chapter-section-paragraph-N
        return f"{chapter_slug}-{section_slug}-p{index}"

    def generate_deep_link_url(self, file_path: str, paragraph_id: str) -> str:
        """
        Generate deep link URL to exact paragraph in Docusaurus.

        Args:
            file_path: Relative file path from docs/
            paragraph_id: Paragraph identifier

        Returns:
            Deep link URL
        """
        # Convert file path to URL path (remove .md/.mdx extension)
        file_path_no_ext = re.sub(r"\.(md|mdx)$", "", file_path)

        # Build URL: {base_url}/docs/{file_path}#{paragraph_id}
        return f"{self.base_url}/docs/{file_path_no_ext}#{paragraph_id}"

    def _slugify(self, text: Optional[str]) -> str:
        """
        Convert text to URL-safe slug.

        Args:
            text: Text to slugify

        Returns:
            Slugified text
        """
        if not text:
            return "unknown"

        # Convert to lowercase
        slug = text.lower()

        # Replace spaces and special characters with hyphens
        slug = re.sub(r"[^\w\s-]", "", slug)
        slug = re.sub(r"[-\s]+", "-", slug)

        # Remove leading/trailing hyphens
        slug = slug.strip("-")

        return slug or "unknown"


def load_book_files(book_path: str) -> List[BookContent]:
    """
    Convenience function to load all book files.

    Args:
        book_path: Path to the Docusaurus docs/ directory

    Returns:
        List of BookContent objects
    """
    import os

    base_url = os.getenv("DOCUSAURUS_BASE_URL", "http://localhost:3000")
    loader = ContentLoader(base_url=base_url)
    return loader.load_book_files(book_path)
