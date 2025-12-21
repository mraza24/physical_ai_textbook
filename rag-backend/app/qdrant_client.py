"""
Qdrant client management for vector search operations.
Provides singleton Qdrant client instance for the application.
"""
import os
from typing import Optional
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Global Qdrant client
_client: Optional[QdrantClient] = None

def get_qdrant_client() -> QdrantClient:
    """
    Get or create the Qdrant client instance.
    Returns the global Qdrant client, creating it if necessary.
    """
    global _client

    if _client is None:
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or qdrant_url == "REPLACE_WITH_YOUR_QDRANT_CLUSTER_URL":
            raise ValueError(
                "QDRANT_URL not configured. "
                "Please update .env with your Qdrant Cloud cluster URL."
            )

        if not qdrant_api_key or qdrant_api_key == "REPLACE_WITH_YOUR_QDRANT_API_KEY":
            raise ValueError(
                "QDRANT_API_KEY not configured. "
                "Please update .env with your Qdrant API key."
            )

        print("Initializing Qdrant client...")

        _client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )

        print("✅ Qdrant client initialized successfully")

    return _client

def close_qdrant_client():
    """Close the Qdrant client connection."""
    global _client

    if _client is not None:
        _client.close()
        _client = None
        print("✅ Qdrant client closed")
