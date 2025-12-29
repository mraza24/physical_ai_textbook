"""
Qdrant Vector Database Service
Handles vector storage and semantic search operations
Compatible with qdrant-client >= 1.12.0
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
        Initialize Qdrant client with URL sanitization and validation

        Required environment variables:
            - QDRANT_URL: URL of Qdrant cluster (e.g., https://your-cluster.qdrant.io)
            - QDRANT_API_KEY: API key for authentication
        """
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = "textbook_chunks"  # Collection name for textbook embeddings

        # Debug: Print what was loaded (masked for security)
        logger.info("=" * 60)
        logger.info("Qdrant Service Initialization")
        logger.info("=" * 60)

        if not self.url:
            logger.error("❌ QDRANT_URL is not set in environment variables")
            logger.error("💡 Add to .env: QDRANT_URL=https://your-cluster.qdrant.io")
            raise ValueError("QDRANT_URL must be set in environment variables")

        if not self.api_key:
            logger.error("❌ QDRANT_API_KEY is not set in environment variables")
            logger.error("💡 Add to .env: QDRANT_API_KEY=your-api-key-here")
            raise ValueError("QDRANT_API_KEY must be set in environment variables")

        # Sanitize URL - remove trailing slashes and common suffixes
        original_url = self.url
        self.url = self._sanitize_url(self.url)

        if self.url != original_url:
            logger.info(f"📝 Sanitized URL: {original_url} → {self.url}")

        # Mask sensitive data for logging
        masked_url = self.url
        masked_key = self.api_key[:8] + "..." if len(self.api_key) > 8 else "***"

        logger.info(f"✅ QDRANT_URL loaded: {masked_url}")
        logger.info(f"✅ QDRANT_API_KEY loaded: {masked_key}")
        logger.info(f"📦 Collection name: {self.collection_name}")

        try:
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
            )
            logger.info(f"✅ Connected to Qdrant cluster at {self.url}")
            logger.info("=" * 60)
        except Exception as e:
            logger.error("=" * 60)
            logger.error(f"❌ Failed to connect to Qdrant: {e}")

            # Provide specific guidance for 403 errors
            if "403" in str(e) or "Forbidden" in str(e):
                logger.error("")
                logger.error("🔍 403 Forbidden Error Detected")
                logger.error("=" * 60)
                logger.error("This usually means:")
                logger.error("1. ❌ Your QDRANT_API_KEY is invalid or expired")
                logger.error("2. ❌ Your Qdrant cluster is paused/deleted on Qdrant Cloud")
                logger.error("3. ❌ The API key doesn't have access to this cluster")
                logger.error("")
                logger.error("💡 Solutions:")
                logger.error("  • Check https://cloud.qdrant.io to verify cluster is active")
                logger.error("  • Regenerate API key in Qdrant Cloud dashboard")
                logger.error("  • Update .env with new credentials")
                logger.error("=" * 60)

            raise

    @staticmethod
    def _sanitize_url(url: str) -> str:
        """
        Sanitize Qdrant URL by removing trailing slashes and common suffixes

        Args:
            url: Raw URL from environment variable

        Returns:
            Cleaned URL suitable for Qdrant client

        Examples:
            https://cluster.qdrant.io/ → https://cluster.qdrant.io
            https://cluster.qdrant.io/dashboard → https://cluster.qdrant.io
            https://cluster.qdrant.io:6333 → https://cluster.qdrant.io:6333
        """
        # Remove trailing slashes
        url = url.rstrip('/')

        # Remove common dashboard/UI suffixes
        suffixes_to_remove = ['/dashboard', '/console', '/ui']
        for suffix in suffixes_to_remove:
            if url.endswith(suffix):
                url = url[:-len(suffix)]
                break

        return url

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.3
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant using the latest API

        Args:
            query_vector: Query embedding vector (from Cohere)
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (0.0-1.0, default: 0.3)

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
            # ⚠️ IMPORTANT: Using query_points() for qdrant-client >= 1.12.0
            # The old .search() method was deprecated

            search_result = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
                with_payload=True,  # Include payload in results
                with_vectors=False,  # Don't return vectors (saves bandwidth)
            )

            # Format results - query_points returns QueryResponse with .points attribute
            results = []
            for point in search_result.points:
                results.append({
                    "id": point.id,
                    "score": point.score,
                    "payload": point.payload,
                })

            logger.info(f"Found {len(results)} results for query (score_threshold={score_threshold})")
            return results

        except AttributeError as e:
            # Fallback for older qdrant-client versions
            logger.warning(f"query_points not available, trying legacy search(): {e}")
            try:
                search_result = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_vector,
                    limit=top_k,
                    score_threshold=score_threshold,
                )

                results = []
                for hit in search_result:
                    results.append({
                        "id": hit.id,
                        "score": hit.score,
                        "payload": hit.payload,
                    })

                logger.info(f"Found {len(results)} results using legacy search")
                return results

            except Exception as legacy_error:
                logger.error(f"Both query_points and search failed: {legacy_error}")
                raise

        except Exception as e:
            logger.error(f"Qdrant search failed: {e}")
            raise

    def create_collection(self, vector_size: int = 1024):
        """
        Create a new collection in Qdrant (checks if exists first to avoid 409 errors)

        Args:
            vector_size: Dimension of embeddings (default: 1024 for Cohere embed-multilingual-v3.0)

        Note:
            This is typically called during initial setup or ingestion.
            For production, create collections via dashboard or deployment script.
        """
        logger.info("📦 Setting up Qdrant collection...")

        try:
            # Connection test: Verify authentication by fetching cluster info
            logger.info("🔍 Testing Qdrant connection and authentication...")
            try:
                cluster_info = self.client.get_collections()
                logger.info(f"✅ Authentication successful - found {len(cluster_info.collections)} existing collections")
            except Exception as conn_error:
                if "403" in str(conn_error) or "Forbidden" in str(conn_error):
                    logger.error("")
                    logger.error("=" * 60)
                    logger.error("❌ 403 FORBIDDEN ERROR - Authentication Failed")
                    logger.error("=" * 60)
                    logger.error("Your Qdrant API key is being rejected by the server.")
                    logger.error("")
                    logger.error("🔍 Common Causes:")
                    logger.error("  1. ❌ API key is invalid or expired")
                    logger.error("  2. ❌ Qdrant cluster is paused or deleted")
                    logger.error("  3. ❌ API key doesn't match this cluster")
                    logger.error("  4. ❌ Wrong cluster URL (check for typos)")
                    logger.error("")
                    logger.error("💡 How to Fix:")
                    logger.error("  Step 1: Visit https://cloud.qdrant.io")
                    logger.error("  Step 2: Check if your cluster is ACTIVE (not paused)")
                    logger.error("  Step 3: Go to API Keys → Copy your key")
                    logger.error("  Step 4: Update .env:")
                    logger.error(f"          QDRANT_URL={self.url}")
                    logger.error("          QDRANT_API_KEY=<your-new-key>")
                    logger.error("  Step 5: Re-run the script")
                    logger.error("=" * 60)
                    raise ValueError("Qdrant authentication failed with 403 Forbidden. Check API key and cluster status.")
                else:
                    logger.error(f"❌ Connection test failed: {conn_error}")
                    raise

            # Check if collection already exists
            existing_collections = self.client.get_collections()
            collection_exists = any(
                col.name == self.collection_name
                for col in existing_collections.collections
            )

            if collection_exists:
                logger.info(f"✅ Collection '{self.collection_name}' already exists, skipping creation")
                return

            # Create collection if it doesn't exist
            logger.info(f"📝 Creating new collection '{self.collection_name}' with vector_size={vector_size}...")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=vector_size,
                    distance=Distance.COSINE,  # Cosine similarity for semantic search
                ),
            )
            logger.info(f"✅ Successfully created collection: {self.collection_name}")

        except ValueError:
            # Re-raise ValueError from 403 handling without wrapping
            raise

        except Exception as e:
            logger.error(f"❌ Collection creation/check failed: {e}")

            # Provide specific guidance for 403 errors during collection operations
            if "403" in str(e) or "Forbidden" in str(e):
                logger.error("")
                logger.error("💡 This 403 error occurred during collection setup.")
                logger.error("   Check that your API key has permission to create collections.")

            raise

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
            return 
        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise
