"""
Embedding service for generating semantic embeddings using Cohere API.

Handles:
- Batch processing of chunks for embedding generation
- Cohere Embed API integration (embed-english-v3.0)
- Retry logic for transient API failures
- Rate limiting for API quotas
"""
import time
from typing import List, Tuple
import cohere

from app.models.chunk import Chunk
from app.cohere_client import get_cohere_client


class EmbeddingService:
    """Generates semantic embeddings for text chunks using Cohere API."""

    # Cohere API limits
    MAX_BATCH_SIZE = 96  # Maximum texts per request
    MAX_RETRIES = 3  # Number of retries for failed requests
    RETRY_DELAY = 1  # Initial delay in seconds between retries

    # Rate limiting (Cohere free tier: 100 calls/min)
    MAX_CALLS_PER_MINUTE = 100
    RATE_LIMIT_WINDOW = 60  # seconds

    def __init__(self):
        """Initialize embedding service."""
        self.client = get_cohere_client()
        self.model = "embed-english-v3.0"  # Cohere embedding model
        self.input_type = "search_document"  # For indexing documents

        # Rate limiting tracking
        self.call_timestamps = []

    def _check_rate_limit(self):
        """
        Check and enforce rate limiting.

        Sleeps if necessary to respect Cohere free tier limits (100 calls/min).
        """
        current_time = time.time()

        # Remove timestamps older than the rate limit window
        self.call_timestamps = [
            ts for ts in self.call_timestamps
            if current_time - ts < self.RATE_LIMIT_WINDOW
        ]

        # If we've hit the rate limit, wait
        if len(self.call_timestamps) >= self.MAX_CALLS_PER_MINUTE:
            oldest_call = self.call_timestamps[0]
            sleep_time = self.RATE_LIMIT_WINDOW - (current_time - oldest_call)

            if sleep_time > 0:
                print(f"Rate limit reached. Sleeping for {sleep_time:.2f}s...")
                time.sleep(sleep_time)

                # Clean up old timestamps after sleeping
                current_time = time.time()
                self.call_timestamps = [
                    ts for ts in self.call_timestamps
                    if current_time - ts < self.RATE_LIMIT_WINDOW
                ]

        # Record this call
        self.call_timestamps.append(current_time)

    def generate_embeddings(
        self, chunks: List[Chunk]
    ) -> List[Tuple[Chunk, List[float]]]:
        """
        Generate embeddings for a list of chunks.

        Processes chunks in batches according to Cohere API limits and
        includes retry logic for transient failures.

        Args:
            chunks: List of Chunk objects to generate embeddings for

        Returns:
            List of (Chunk, embedding_vector) tuples
        """
        if not chunks:
            return []

        results = []

        # Process in batches
        for i in range(0, len(chunks), self.MAX_BATCH_SIZE):
            batch = chunks[i : i + self.MAX_BATCH_SIZE]

            try:
                batch_embeddings = self._generate_batch_embeddings(batch)
                results.extend(zip(batch, batch_embeddings))
            except Exception as e:
                print(f"Error generating embeddings for batch {i//self.MAX_BATCH_SIZE + 1}: {e}")
                raise

        return results

    def _generate_batch_embeddings(self, chunks: List[Chunk]) -> List[List[float]]:
        """
        Generate embeddings for a batch of chunks with retry logic.

        Args:
            chunks: List of Chunk objects (max 96)

        Returns:
            List of embedding vectors

        Raises:
            Exception: If all retries fail
        """
        texts = [chunk.text_content for chunk in chunks]

        for attempt in range(self.MAX_RETRIES):
            try:
                # Check rate limit before making API call
                self._check_rate_limit()

                # Call Cohere Embed API
                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type=self.input_type,
                )

                # Extract embeddings from response
                embeddings = response.embeddings

                return embeddings

            except cohere.errors.TooManyRequestsError as e:
                # Rate limit exceeded - wait longer
                if attempt < self.MAX_RETRIES - 1:
                    delay = self.RETRY_DELAY * (2**attempt) * 2  # Exponential backoff, doubled
                    print(f"Rate limit exceeded. Retrying in {delay} seconds...")
                    time.sleep(delay)
                else:
                    print(f"Rate limit exceeded after {self.MAX_RETRIES} attempts")
                    raise

            except (
                cohere.errors.InternalServerError,
                cohere.errors.ServiceUnavailableError,
            ) as e:
                # Transient server errors - retry with exponential backoff
                if attempt < self.MAX_RETRIES - 1:
                    delay = self.RETRY_DELAY * (2**attempt)
                    print(f"Server error: {e}. Retrying in {delay} seconds...")
                    time.sleep(delay)
                else:
                    print(f"Server error persisted after {self.MAX_RETRIES} attempts")
                    raise

            except Exception as e:
                # Other errors - don't retry
                print(f"Unexpected error generating embeddings: {e}")
                raise

        # Should not reach here
        raise Exception(f"Failed to generate embeddings after {self.MAX_RETRIES} attempts")

    def generate_query_embedding(self, query_text: str) -> List[float]:
        """
        Generate embedding for a search query.

        Args:
            query_text: Query text

        Returns:
            Embedding vector

        Raises:
            Exception: If embedding generation fails
        """
        for attempt in range(self.MAX_RETRIES):
            try:
                # Check rate limit before making API call
                self._check_rate_limit()

                # Use search_query input type for queries
                response = self.client.embed(
                    texts=[query_text],
                    model=self.model,
                    input_type="search_query",  # Different input type for queries
                )

                return response.embeddings[0]

            except cohere.errors.TooManyRequestsError as e:
                if attempt < self.MAX_RETRIES - 1:
                    delay = self.RETRY_DELAY * (2**attempt) * 2
                    print(f"Rate limit exceeded. Retrying in {delay} seconds...")
                    time.sleep(delay)
                else:
                    raise

            except (
                cohere.errors.InternalServerError,
                cohere.errors.ServiceUnavailableError,
            ) as e:
                if attempt < self.MAX_RETRIES - 1:
                    delay = self.RETRY_DELAY * (2**attempt)
                    print(f"Server error: {e}. Retrying in {delay} seconds...")
                    time.sleep(delay)
                else:
                    raise

            except Exception as e:
                print(f"Unexpected error generating query embedding: {e}")
                raise

        raise Exception(f"Failed to generate query embedding after {self.MAX_RETRIES} attempts")


def generate_embeddings(chunks: List[Chunk]) -> List[Tuple[Chunk, List[float]]]:
    """
    Convenience function to generate embeddings for chunks.

    Args:
        chunks: List of Chunk objects

    Returns:
        List of (Chunk, embedding_vector) tuples
    """
    service = EmbeddingService()
    return service.generate_embeddings(chunks)


def generate_query_embedding(query_text: str) -> List[float]:
    """
    Convenience function to generate embedding for a query.

    Args:
        query_text: Query text

    Returns:
        Embedding vector
    """
    service = EmbeddingService()
    return service.generate_query_embedding(query_text)
