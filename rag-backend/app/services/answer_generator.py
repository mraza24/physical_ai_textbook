"""
Answer generator service using Cohere Chat API.

Handles:
- Cohere Chat API integration
- System prompt for preventing hallucination
- Temperature configuration
- Streaming support (optional)
"""
import os
from typing import Tuple
import cohere

from app.cohere_client import get_cohere_client


class AnswerGenerator:
    """Generates answers using Cohere Chat API."""

    def __init__(self):
        """Initialize answer generator."""
        self.client = get_cohere_client()

        # Get configuration from environment
        self.temperature_min = float(os.getenv("TEMPERATURE_MIN", "0.3"))
        self.temperature_max = float(os.getenv("TEMPERATURE_MAX", "0.5"))
        self.temperature = (self.temperature_min + self.temperature_max) / 2
        self.max_tokens = int(os.getenv("MAX_RESPONSE_TOKENS", "500"))

    def generate_answer(
        self, assembled_context: str, query_text: str
    ) -> Tuple[str, int]:
        """
        Generate an answer using Cohere Chat API.

        Uses a system prompt that prevents hallucination by requiring the model
        to answer only from the provided context.

        Args:
            assembled_context: The formatted context with retrieved chunks
            query_text: The user's query

        Returns:
            Tuple of (answer_text, token_count)

        Raises:
            Exception: If answer generation fails
        """
        # System prompt to prevent hallucination
        system_prompt = (
            "You are a helpful teaching assistant for an AI textbook. "
            "Answer questions using ONLY the information provided in the context. "
            "If the context doesn't contain enough information, say so explicitly. "
            "Be concise and clear in your explanations. "
            "Never make up information or use knowledge outside the provided context."
        )

        # Combine context and query into the message
        message = f"""{assembled_context}

Question: {query_text}

Please answer the question based only on the context provided above."""

        try:
            # Call Cohere Chat API
            response = self.client.chat(
                message=message,
                preamble=system_prompt,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )

            answer_text = response.text
            token_count = len(answer_text.split())  # Approximate token count

            return answer_text, token_count

        except Exception as e:
            print(f"Error generating answer: {e}")
            raise


def generate_answer(
    assembled_context: str, query_text: str
) -> Tuple[str, int]:
    """
    Convenience function to generate an answer.

    Args:
        assembled_context: The formatted context
        query_text: The user's query

    Returns:
        Tuple of (answer_text, token_count)
    """
    generator = AnswerGenerator()
    return generator.generate_answer(assembled_context, query_text)
