"""
Cohere client management for embedding and generation operations.
Provides singleton Cohere client instance for the application.
"""
import os
from typing import Optional
import cohere
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Global Cohere client
_client: Optional[cohere.Client] = None

def get_cohere_client() -> cohere.Client:
    """
    Get or create the Cohere client instance.
    Returns the global Cohere client, creating it if necessary.
    """
    global _client

    if _client is None:
        cohere_api_key = os.getenv("COHERE_API_KEY")

        if not cohere_api_key or cohere_api_key == "REPLACE_WITH_YOUR_COHERE_API_KEY":
            raise ValueError(
                "COHERE_API_KEY not configured. "
                "Please update .env with your Cohere API key."
            )

        print("Initializing Cohere client...")

        _client = cohere.Client(api_key=cohere_api_key)

        print("✅ Cohere client initialized successfully")

    return _client
