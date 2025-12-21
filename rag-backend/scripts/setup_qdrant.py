#!/usr/bin/env python3
"""
Qdrant collection setup script for RAG Chatbot.
Creates the textbook_chunks collection with proper schema and indexes.
"""
import os
import sys
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import (
    VectorParams,
    Distance,
    PayloadSchemaType,
    TextIndexParams,
    TextIndexType,
    TokenizerType,
)

# Load environment variables
load_dotenv()

COLLECTION_NAME = "textbook_chunks"
VECTOR_SIZE = 1024  # Cohere embed-english-v3.0 dimension
DISTANCE_METRIC = Distance.COSINE

def create_collection(client: QdrantClient) -> None:
    """Create the textbook_chunks collection with vector configuration."""
    print(f"Creating collection: {COLLECTION_NAME}")

    # Check if collection already exists
    collections = client.get_collections().collections
    if any(c.name == COLLECTION_NAME for c in collections):
        print(f"⚠️  Collection '{COLLECTION_NAME}' already exists. Recreating...")
        client.delete_collection(COLLECTION_NAME)

    # Create collection with vector configuration
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
            size=VECTOR_SIZE,
            distance=DISTANCE_METRIC,
        ),
    )

    print(f"✅ Collection '{COLLECTION_NAME}' created with:")
    print(f"   - Vector size: {VECTOR_SIZE}")
    print(f"   - Distance metric: {DISTANCE_METRIC.value}")

def create_payload_schema(client: QdrantClient) -> None:
    """Define payload schema with field types and indexes."""
    print("\nConfiguring payload schema...")

    # Define schema for keyword fields (exact match, filtering)
    keyword_fields = [
        "chunk_id",
        "chapter",
        "section",
        "content_type",
        "paragraph_id",
        "deep_link_url",
        "book_version",
    ]

    for field in keyword_fields:
        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name=field,
            field_schema=PayloadSchemaType.KEYWORD,
        )
        print(f"   ✅ Created KEYWORD index: {field}")

    # Define schema for integer fields
    integer_fields = ["token_count", "overlap_start", "overlap_end"]

    for field in integer_fields:
        client.create_payload_index(
            collection_name=COLLECTION_NAME,
            field_name=field,
            field_schema=PayloadSchemaType.INTEGER,
        )
        print(f"   ✅ Created INTEGER index: {field}")

def create_bm25_index(client: QdrantClient) -> None:
    """Create BM25 full-text search index for hybrid search."""
    print("\nCreating BM25 index for hybrid search...")

    # Create text index on text_content field
    client.create_payload_index(
        collection_name=COLLECTION_NAME,
        field_name="text_content",
        field_schema=TextIndexParams(
            type=TextIndexType.TEXT,
            tokenizer=TokenizerType.WORD,
            min_token_len=2,
            max_token_len=20,
            lowercase=True,
        ),
    )

    print("   ✅ Created BM25 text index on 'text_content' field")
    print("      - Tokenizer: word")
    print("      - Min token length: 2")
    print("      - Max token length: 20")
    print("      - Lowercase: enabled")

def main():
    """Initialize Qdrant collection with schema and indexes."""
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    # Validate configuration
    if not qdrant_url or qdrant_url == "REPLACE_WITH_YOUR_QDRANT_CLUSTER_URL":
        print("❌ ERROR: QDRANT_URL not configured in .env file")
        print("Please update .env with your Qdrant Cloud cluster URL")
        sys.exit(1)

    if not qdrant_api_key or qdrant_api_key == "REPLACE_WITH_YOUR_QDRANT_API_KEY":
        print("❌ ERROR: QDRANT_API_KEY not configured in .env file")
        print("Please update .env with your Qdrant API key")
        sys.exit(1)

    print("=" * 60)
    print("Qdrant Collection Setup for RAG Chatbot")
    print("=" * 60)
    print()

    try:
        # Initialize Qdrant client
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )

        # Create collection
        create_collection(client)

        # Configure payload schema
        create_payload_schema(client)

        # Create BM25 index
        create_bm25_index(client)

        print("\n" + "=" * 60)
        print("🎉 Qdrant setup completed successfully!")
        print("=" * 60)
        print(f"\nCollection '{COLLECTION_NAME}' is ready for ingestion.")

    except Exception as e:
        print(f"\n❌ Error setting up Qdrant: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
