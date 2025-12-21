"""
Vector store service for Qdrant operations.

Handles:
- Batch upsert of chunks with embeddings to Qdrant
- Hybrid search (semantic + BM25) for retrieval
- Collection management and querying
"""
import os
from typing import List
from qdrant_client.models import PointStruct, SearchParams

from app.models.chunk import Chunk
from app.models.retrieved_chunk import RetrievedChunk
from app.qdrant_client import get_qdrant_client


class VectorStore:
    """Manages vector operations with Qdrant."""

    COLLECTION_NAME = "textbook_chunks"
    BATCH_SIZE = 100  # Batch size for upsert operations

    def __init__(self):
        """Initialize vector store."""
        self.client = get_qdrant_client()
        self.collection_name = self.COLLECTION_NAME

    def upsert_chunks(
        self, chunks: List[Chunk], embeddings: List[List[float]]
    ) -> int:
        """
        Upsert chunks with embeddings to Qdrant collection.

        Processes in batches for efficient upload.

        Args:
            chunks: List of Chunk objects
            embeddings: List of embedding vectors (same order as chunks)

        Returns:
            Number of chunks upserted

        Raises:
            ValueError: If chunks and embeddings length mismatch
        """
        if len(chunks) != len(embeddings):
            raise ValueError(
                f"Chunks and embeddings length mismatch: {len(chunks)} vs {len(embeddings)}"
            )

        if not chunks:
            return 0

        total_upserted = 0

        # Process in batches
        for i in range(0, len(chunks), self.BATCH_SIZE):
            batch_chunks = chunks[i : i + self.BATCH_SIZE]
            batch_embeddings = embeddings[i : i + self.BATCH_SIZE]

            # Create points for this batch
            points = []
            for chunk, embedding in zip(batch_chunks, batch_embeddings):
                point = PointStruct(
                    id=chunk.chunk_id,  # Use chunk_id as point ID
                    vector=embedding,
                    payload={
                        "chunk_id": chunk.chunk_id,
                        "text_content": chunk.text_content,
                        "chapter": chunk.chapter,
                        "section": chunk.section or "",
                        "subsection": chunk.subsection or "",
                        "content_type": chunk.content_type,
                        "paragraph_id": chunk.paragraph_id,
                        "deep_link_url": chunk.deep_link_url,
                        "token_count": chunk.token_count,
                        "overlap_start": chunk.overlap_start,
                        "overlap_end": chunk.overlap_end,
                        "book_version": chunk.book_version,
                    },
                )
                points.append(point)

            # Upsert batch to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            total_upserted += len(points)
            print(f"Upserted batch {i//self.BATCH_SIZE + 1}: {len(points)} chunks")

        print(f"Total upserted: {total_upserted} chunks")
        return total_upserted

    def hybrid_search(
        self,
        query_embedding: List[float],
        query_text: str,
        top_k: int = 5,
        similarity_threshold: float = 0.7,
        semantic_weight: float = 0.7,
        keyword_weight: float = 0.3,
    ) -> List[RetrievedChunk]:
        """
        Perform hybrid search combining semantic similarity and BM25 keyword matching.

        Note: In qdrant-client 1.7.3, true hybrid search with RRF fusion is not available.
        This implementation uses semantic search only. For full hybrid search, upgrade to
        qdrant-client >= 1.8.0 or implement custom score fusion.

        Args:
            query_embedding: Query embedding vector
            query_text: Query text for BM25 search
            top_k: Number of top results to return (default: 5)
            similarity_threshold: Minimum similarity score (default: 0.7)
            semantic_weight: Weight for semantic similarity (default: 0.7)
            keyword_weight: Weight for BM25 keyword matching (default: 0.3)

        Returns:
            List of RetrievedChunk objects sorted by relevance
        """
        # Get similarity threshold from environment if available
        env_threshold = os.getenv("SIMILARITY_THRESHOLD")
        if env_threshold:
            similarity_threshold = float(env_threshold)

        # Optimize search performance with search params
        # ef (exploration factor) controls accuracy vs speed tradeoff
        # Higher ef = more accurate but slower (default: 128)
        # Lower ef = faster but may miss some results
        # For <500ms latency, use ef=100-150
        search_params = SearchParams(
            hnsw_ef=128,  # Balanced accuracy/speed for <500ms retrieval
            exact=False,  # Use HNSW index (faster)
        )

        # Perform semantic search
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=similarity_threshold,
            with_payload=True,
            search_params=search_params,
        )

        # Convert results to RetrievedChunk objects
        retrieved_chunks = []

        for scored_point in search_results:
            payload = scored_point.payload

            retrieved_chunk = RetrievedChunk(
                chunk_id=payload["chunk_id"],
                text_content=payload["text_content"],
                chapter=payload["chapter"],
                section=payload.get("section", ""),
                content_type=payload["content_type"],
                paragraph_id=payload["paragraph_id"],
                deep_link_url=payload["deep_link_url"],
                similarity_score=scored_point.score,
            )

            retrieved_chunks.append(retrieved_chunk)

        # Already sorted by similarity score (descending) from Qdrant
        return retrieved_chunks

    def get_collection_info(self) -> dict:
        """
        Get information about the Qdrant collection.

        Returns:
            Dictionary with collection information
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "collection_name": self.collection_name,
                "points_count": collection_info.points_count,
                "status": collection_info.status,
            }
        except Exception as e:
            return {
                "collection_name": self.collection_name,
                "error": str(e),
            }


def upsert_chunks(chunks: List[Chunk], embeddings: List[List[float]]) -> int:
    """
    Convenience function to upsert chunks.

    Args:
        chunks: List of Chunk objects
        embeddings: List of embedding vectors

    Returns:
        Number of chunks upserted
    """
    store = VectorStore()
    return store.upsert_chunks(chunks, embeddings)


def hybrid_search(
    query_embedding: List[float],
    query_text: str,
    top_k: int = 5,
    similarity_threshold: float = 0.7,
) -> List[RetrievedChunk]:
    """
    Convenience function for hybrid search.

    Args:
        query_embedding: Query embedding vector
        query_text: Query text for BM25 search
        top_k: Number of results to return
        similarity_threshold: Minimum similarity score

    Returns:
        List of RetrievedChunk objects
    """
    store = VectorStore()
    return store.hybrid_search(
        query_embedding=query_embedding,
        query_text=query_text,
        top_k=top_k,
        similarity_threshold=similarity_threshold,
    )
