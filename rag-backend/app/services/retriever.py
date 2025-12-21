"""
Retriever service for fetching relevant chunks from vector store.

Handles:
- Query embedding generation
- Hybrid search (semantic + BM25)
- Similarity filtering and deduplication
"""
import os
from typing import List, Optional

from app.models.retrieved_chunk import RetrievedChunk
from app.services.embedding_service import generate_query_embedding
from app.services.vector_store import hybrid_search as vector_hybrid_search


def retrieve_chunks(
    query_text: str,
    selected_text: Optional[str] = None,
    top_k: int = 5,
) -> List[RetrievedChunk]:
    """
    Retrieve relevant chunks for a query using hybrid search.

    Steps:
    1. Generate query embedding using Cohere
    2. Perform hybrid search (70% semantic + 30% BM25)
    3. Filter by similarity threshold
    4. Deduplicate by chunk_id

    Args:
        query_text: The user's query
        selected_text: Optional selected text (not used in current implementation)
        top_k: Number of chunks to retrieve (default: 5)

    Returns:
        List of RetrievedChunk objects sorted by relevance
    """
    # Get configuration from environment
    top_k_env = os.getenv("TOP_K_CHUNKS")
    if top_k_env:
        top_k = int(top_k_env)

    similarity_threshold = float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))

    # Step 1: Generate query embedding
    query_embedding = generate_query_embedding(query_text)

    # Step 2: Perform hybrid search
    retrieved_chunks = vector_hybrid_search(
        query_embedding=query_embedding,
        query_text=query_text,
        top_k=top_k,
        similarity_threshold=similarity_threshold,
    )

    # Step 3: Filter and deduplicate
    # Qdrant already filters by threshold and returns unique results
    # But we deduplicate again to be safe
    seen_ids = set()
    deduplicated = []

    for chunk in retrieved_chunks:
        if chunk.chunk_id not in seen_ids:
            seen_ids.add(chunk.chunk_id)
            deduplicated.append(chunk)

    # Already sorted by similarity score in vector_store.py
    return deduplicated
