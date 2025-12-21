"""
Context assembler service for formatting retrieved chunks into prompts.

Handles:
- Context formatting with selected text priority
- Prompt structuring for Cohere Chat API
- Token counting and truncation
"""
from typing import List, Tuple, Optional
import tiktoken

from app.models.retrieved_chunk import RetrievedChunk


class ContextAssembler:
    """Assembles context from retrieved chunks for LLM generation."""

    # Token limits for Cohere Chat API
    MAX_CONTEXT_TOKENS = 3000  # Leave room for query and response

    def __init__(self):
        """Initialize context assembler."""
        self.encoder = tiktoken.get_encoding("cl100k_base")

    def assemble_context(
        self,
        query_text: str,
        retrieved_chunks: List[RetrievedChunk],
        selected_text: Optional[str] = None,
    ) -> Tuple[str, List[RetrievedChunk]]:
        """
        Assemble context from query, retrieved chunks, and optional selected text.

        Priority order:
        1. Selected text (if provided)
        2. Retrieved chunks (sorted by relevance)

        Args:
            query_text: The user's query
            retrieved_chunks: List of retrieved chunks sorted by relevance
            selected_text: Optional text selected by user

        Returns:
            Tuple of (formatted_context, chunks_used)
        """
        context_parts = []
        chunks_used = []
        total_tokens = 0

        # Add selected text with highest priority
        if selected_text:
            selected_tokens = self.count_tokens(selected_text)

            if selected_tokens < self.MAX_CONTEXT_TOKENS:
                context_parts.append(f"Selected Passage:\n{selected_text}\n")
                total_tokens += selected_tokens

        # Add retrieved chunks
        if retrieved_chunks:
            context_parts.append("Related Sections from Textbook:")

            for i, chunk in enumerate(retrieved_chunks, start=1):
                chunk_text = f"\n{i}. {chunk.text_content}\n   (Source: {chunk.chapter} - {chunk.section})"
                chunk_tokens = self.count_tokens(chunk_text)

                # Check if adding this chunk would exceed limit
                if total_tokens + chunk_tokens > self.MAX_CONTEXT_TOKENS:
                    break

                context_parts.append(chunk_text)
                chunks_used.append(chunk)
                total_tokens += chunk_tokens

        # Format final context
        context = "\n".join(context_parts)

        return context, chunks_used

    def structure_prompt(
        self, query_text: str, assembled_context: str
    ) -> str:
        """
        Structure the final prompt for Cohere Chat API.

        Args:
            query_text: The user's query
            assembled_context: The assembled context

        Returns:
            Formatted prompt string
        """
        prompt = f"""Context from textbook:
{assembled_context}

Question: {query_text}

Instructions: Answer the question using ONLY the information provided in the context above. If the context doesn't contain enough information to answer the question, respond with: "I cannot find sufficient information in the textbook to answer this question accurately."
"""

        return prompt

    def count_tokens(self, text: str) -> int:
        """
        Count tokens in text.

        Args:
            text: Text to count

        Returns:
            Number of tokens
        """
        return len(self.encoder.encode(text))


def assemble_context(
    query_text: str,
    retrieved_chunks: List[RetrievedChunk],
    selected_text: Optional[str] = None,
) -> Tuple[str, List[RetrievedChunk]]:
    """
    Convenience function to assemble context.

    Args:
        query_text: The user's query
        retrieved_chunks: List of retrieved chunks
        selected_text: Optional selected text

    Returns:
        Tuple of (formatted_context, chunks_used)
    """
    assembler = ContextAssembler()
    return assembler.assemble_context(query_text, retrieved_chunks, selected_text)
