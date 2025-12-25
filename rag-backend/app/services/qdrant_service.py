"""
Qdrant Vector Database Service
Handles vector storage and semantic search operations
"""

import os
import logging
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter

logger = logging.getLogger(__name__)


class QdrantService:
    """
    Service for interacting with Qdrant vector database

    Usage:
        service = QdrantService()
        results = service.search(query_vector=[0.1, 0.2, ...], top_k=5)
    """

    def __init__(self):
        """
        Initialize Qdrant client

        Required environment variables:
            - QDRANT_URL: URL of Qdrant cluster (e.g., https://your-cluster.qdrant.io)
            - QDRANT_API_KEY: API key for authentication
        """
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = "textbook_chunks"  # Collection name for textbook embeddings

        # ⚠️ USER TODO: Ensure QDRANT_URL and QDRANT_API_KEY are set in .env file
        if not self.url or not self.api_key:
            raise ValueError(
                "QDRANT_URL and QDRANT_API_KEY must be set in environment variables. "
                "Check your .env file and ensure these credentials are configured."
            )

        try:
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
            )
            logger.info(f"Connected to Qdrant cluster at {self.url}")
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {e}")
            raise

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.3
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant

        Args:
            query_vector: Query embedding vector (from Cohere)
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (0.0-1.0, default: 0.7)

        Returns:
            List of search results with payload and scores

        Example:
            results = service.search(
                query_vector=[0.1, 0.2, ...],
                top_k=5,
                score_threshold=0.7
            )
            # Returns: [
            #     {"id": "chunk_123", "score": 0.89, "payload": {"text": "...", "metadata": {...}}},
            #     ...
            # ]
        """
        try:
            # ⚠️ USER TODO: Ensure the collection exists in Qdrant before searching
            # You can create collections using the Qdrant dashboard or API

            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
            )

            # Format results
            results = []
            for hit in search_result:
                results.append({
                    "id": hit.id,
                    "score": hit.score,
                    "payload": hit.payload,
                })

            logger.info(f"Found {len(results)} results for query (score_threshold={score_threshold})")
            return results

        except Exception as e:
            logger.error(f"Qdrant search failed: {e}")
            raise

    def create_collection(self, vector_size: int = 1024):
        """
        Create a new collection in Qdrant

        Args:
            vector_size: Dimension of embeddings (default: 1024 for Cohere embed-english-v3.0)

        Note:
            This is typically called during initial setup or ingestion.
            For production, create collections via dashboard or deployment script.
        """
        try:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE,  # Cosine similarity for semantic search
                ),
            )
            logger.info(f"Created collection: {self.collection_name}")
        except Exception as e:
            logger.warning(f"Collection creation failed (may already exist): {e}")

    def upsert_points(self, points: List[Dict[str, Any]]):
        """
        Insert or update points (vectors) in Qdrant

        Args:
            points: List of point dictionaries with id, vector, and payload

        Example:
            service.upsert_points([
                {
                    "id": "chunk_001",
                    "vector": [0.1, 0.2, ...],
                    "payload": {
                        "text": "Forward kinematics is...",
                        "section_title": "Module 1: Kinematics",
                        "deep_link_url": "/docs/module1/kinematics"
                    }
                }
            ])
        """
        try:
            point_structs = [
                PointStruct(
                    id=p["id"],
                    vector=p["vector"],
                    payload=p.get("payload", {})
                )
                for p in points
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=point_structs,
            )
            logger.info(f"Upserted {len(points)} points to Qdrant")

        except Exception as e:
            logger.error(f"Qdrant upsert failed: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection information (vector count, size, etc.)

        Returns:
            Dictionary with collection metadata
        """
        try:
            info = self.client.get_collection(collection_name=self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise
