"""
Query validator service for input validation and sanitization.

Handles:
- Input sanitization (HTML stripping, whitespace trimming)
- Prompt injection detection
- Language detection (English-only)
- Length limit enforcement
"""
import re
from typing import Tuple, Optional
import html


class QueryValidator:
    """Validates and sanitizes user queries."""

    # Prompt injection patterns to detect
    INJECTION_PATTERNS = [
        r"ignore\s+previous\s+instructions",
        r"ignore\s+all\s+previous",
        r"disregard\s+previous",
        r"you\s+are\s+now",
        r"forget\s+everything",
        r"new\s+instructions",
        r"repeat\s+your\s+instructions",
        r"what\s+are\s+your\s+instructions",
        r"system\s*:",
        r"<\s*system\s*>",
    ]

    # Query length limits
    MIN_QUERY_LENGTH = 10
    MAX_QUERY_LENGTH = 500
    MAX_SELECTED_TEXT_LENGTH = 5000

    def validate_query(
        self, query_text: str, selected_text: Optional[str] = None
    ) -> Tuple[bool, Optional[str]]:
        """
        Validate and sanitize a user query.

        Performs multiple validation checks:
        1. Sanitize input (strip HTML, trim whitespace)
        2. Check for prompt injection patterns
        3. Detect language (English-only)
        4. Enforce length limits

        Args:
            query_text: The user's query text
            selected_text: Optional selected text from the book

        Returns:
            Tuple of (is_valid, error_message)
            - is_valid: True if query passes all checks
            - error_message: Error message if validation fails, None otherwise
        """
        # Step 1: Sanitize input
        query_text = self._sanitize_input(query_text)

        if selected_text:
            selected_text = self._sanitize_input(selected_text)

        # Step 2: Check for prompt injection
        is_safe, injection_error = self._detect_prompt_injection(query_text)
        if not is_safe:
            return False, injection_error

        # Step 3: Detect language
        is_english, language_error = self._detect_language(query_text)
        if not is_english:
            return False, language_error

        # Step 4: Enforce length limits
        is_valid_length, length_error = self._enforce_length_limits(
            query_text, selected_text
        )
        if not is_valid_length:
            return False, length_error

        # All checks passed
        return True, None

    def _sanitize_input(self, text: str) -> str:
        """
        Sanitize input by stripping HTML and trimming whitespace.

        Args:
            text: Input text

        Returns:
            Sanitized text
        """
        # Unescape HTML entities
        text = html.unescape(text)

        # Remove HTML tags
        text = re.sub(r"<[^>]+>", "", text)

        # Trim whitespace
        text = text.strip()

        # Normalize whitespace (replace multiple spaces/newlines with single space)
        text = re.sub(r"\s+", " ", text)

        return text

    def _detect_prompt_injection(self, query_text: str) -> Tuple[bool, Optional[str]]:
        """
        Detect prompt injection attempts.

        Args:
            query_text: The query text to check

        Returns:
            Tuple of (is_safe, error_message)
        """
        query_lower = query_text.lower()

        for pattern in self.INJECTION_PATTERNS:
            if re.search(pattern, query_lower):
                return (
                    False,
                    "I can only answer questions about the textbook content. "
                    "Please rephrase your question.",
                )

        return True, None

    def _detect_language(self, query_text: str) -> Tuple[bool, Optional[str]]:
        """
        Detect if query is in English.

        Uses a simple heuristic: if >30% of characters are non-ASCII,
        likely not English.

        Args:
            query_text: The query text to check

        Returns:
            Tuple of (is_english, error_message)
        """
        if not query_text:
            return False, "Query cannot be empty."

        # Count non-ASCII characters
        non_ascii_count = sum(1 for char in query_text if ord(char) > 127)
        total_chars = len(query_text)

        # If more than 30% non-ASCII, likely not English
        if total_chars > 0 and (non_ascii_count / total_chars) > 0.3:
            return (
                False,
                "I can only answer questions in English. "
                "Please rephrase your question in English.",
            )

        return True, None

    def _enforce_length_limits(
        self, query_text: str, selected_text: Optional[str] = None
    ) -> Tuple[bool, Optional[str]]:
        """
        Enforce length limits on query and selected text.

        Args:
            query_text: The query text
            selected_text: Optional selected text

        Returns:
            Tuple of (is_valid, error_message)
        """
        # Check query length
        if len(query_text) < self.MIN_QUERY_LENGTH:
            return (
                False,
                f"Query is too short. Please provide at least {self.MIN_QUERY_LENGTH} characters.",
            )

        if len(query_text) > self.MAX_QUERY_LENGTH:
            return (
                False,
                f"Query is too long. Please limit your question to {self.MAX_QUERY_LENGTH} characters.",
            )

        # Check selected text length
        if selected_text and len(selected_text) > self.MAX_SELECTED_TEXT_LENGTH:
            return (
                False,
                f"Selected text is too long. Please select less than {self.MAX_SELECTED_TEXT_LENGTH} characters.",
            )

        return True, None


def validate_query(
    query_text: str, selected_text: Optional[str] = None
) -> Tuple[bool, Optional[str]]:
    """
    Convenience function to validate a query.

    Args:
        query_text: The user's query text
        selected_text: Optional selected text from the book

    Returns:
        Tuple of (is_valid, error_message)
    """
    validator = QueryValidator()
    return validator.validate_query(query_text, selected_text)
