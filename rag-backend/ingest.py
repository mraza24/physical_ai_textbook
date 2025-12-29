"""
Diagnostic Textbook Data Ingestion Script
With comprehensive error tracking and minimal batch size for debugging
"""

import uuid
import os
import time
import traceback
from typing import List, Dict, Any
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# DIAGNOSTIC CONFIGURATION - Minimal batching for testing
BATCH_SIZE = 96  # Process 1 chunk at a time for diagnostics
CHUNK_SIZE = 2000  # Characters per chunk
BASE_DELAY = 0  # 20 seconds between batches for diagnostics
MAX_RETRIES = 5  # Maximum retry attempts for 429 errors


def exponential_backoff(attempt: int, base_delay: float = 2.0) -> float:
    """
    Calculate exponential backoff delay

    Args:
        attempt: Current attempt number (0-indexed)
        base_delay: Base delay in seconds

    Returns:
        Delay in seconds (capped at 120s)
    """
    delay = (2 ** attempt) * base_delay
    return min(delay, 120)  # Cap at 2 minutes


def test_cohere_api(cohere: CohereService) -> bool:
    """
    Test Cohere API with a single word to verify connectivity

    Returns:
        True if test passes, False otherwise
    """
    print("=" * 70)
    print("🧪 DIAGNOSTIC TEST 1: Cohere API Connection")
    print("=" * 70)
    print()
    print("Testing Cohere API by embedding single word 'test'...")
    print()

    try:
        # Try to embed a single word
        result = cohere.embed_text("test")

        print("✅ Cohere API Test PASSED")
        print(f"   • Embedding dimension: {len(result)}")
        print(f"   • First 5 values: {result[:5]}")
        print(f"   • Embedding type: {type(result)}")
        print()
        return True

    except Exception as e:
        print("❌ Cohere API Test FAILED")
        print()
        print("Full Raw Error from Cohere:")
        print("-" * 70)
        print(f"Error Type: {type(e).__name__}")
        print(f"Error Message: {str(e)}")
        print()
        print("Full Traceback:")
        print(traceback.format_exc())
        print("-" * 70)
        print()
        print("💡 This indicates the problem is with Cohere API access, not just rate limits.")
        print("   Check your COHERE_API_KEY in .env file.")
        print()
        return False


def process_file(file_path: str, cohere: CohereService, qdrant: QdrantService) -> int:
    """
    Process a single file with diagnostic logging

    Args:
        file_path: Path to the file to process
        cohere: Initialized Cohere service instance
        qdrant: Initialized Qdrant service instance

    Returns:
        Number of chunks successfully uploaded
    """
    # Read file
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            text_content = f.read()
    except Exception as e:
        print(f"❌ Error reading {file_path}: {e}")
        return 0

    if not text_content.strip():
        print(f"⚠️  Skipping empty file: {os.path.basename(file_path)}")
        return 0

    # Chunk the text
    chunks = [text_content[i:i+CHUNK_SIZE] for i in range(0, len(text_content), CHUNK_SIZE)]
    chunks = [c for c in chunks if c.strip()]  # Filter empty chunks

    filename = os.path.basename(file_path)
    print(f"⏳ Processing {len(chunks)} chunks from {filename}...")
    print(f"   Using batch size: {BATCH_SIZE} chunks per request")
    print(f"   Delay between batches: {BASE_DELAY}s")
    print()

    all_points = []
    total_processed = 0

    # Process chunks in batches
    for batch_start in range(0, len(chunks), BATCH_SIZE):
        batch_end = min(batch_start + BATCH_SIZE, len(chunks))
        batch_chunks = chunks[batch_start:batch_end]
        batch_num = (batch_start // BATCH_SIZE) + 1
        total_batches = (len(chunks) + BATCH_SIZE - 1) // BATCH_SIZE

        print(f"   📦 Batch {batch_num}/{total_batches}: {len(batch_chunks)} chunks...")

        # Retry logic with exponential backoff
        success = False
        attempt = 0

        while not success and attempt < MAX_RETRIES:
            try:
                # Generate embeddings for entire batch at once
                embeddings = cohere.embed_batch(
                    texts=batch_chunks,
                    batch_size=len(batch_chunks)
                )

                # Create points for this batch
                for i, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings)):
                    chunk_index = batch_start + i
                    point = {
                        "id": str(uuid.uuid4()),
                        "vector": embedding,
                        "payload": {
                            "text": chunk,
                            "source": filename,
                            "section_title": filename.replace('.md', '').replace('.txt', ''),
                            "deep_link_url": f"/docs/{filename.replace('.md', '')}",
                            "metadata": {
                                "chunk_index": chunk_index,
                                "total_chunks": len(chunks)
                            }
                        }
                    }
                    all_points.append(point)

                    # DIAGNOSTIC: Print first point structure
                    if len(all_points) == 1:
                        print()
                        print("=" * 70)
                        print("🧪 DIAGNOSTIC TEST 2: Point Structure Check")
                        print("=" * 70)
                        print()
                        print("First generated Point object:")
                        print("-" * 70)
                        print(f"ID: {point['id']}")
                        print(f"Vector dimension: {len(point['vector'])}")
                        print(f"Vector type: {type(point['vector'])}")
                        print(f"Vector first 3 values: {point['vector'][:3]}")
                        print()
                        print("Payload structure:")
                        print(f"  - source: {point['payload']['source']}")
                        print(f"  - section_title: {point['payload']['section_title']}")
                        print(f"  - deep_link_url: {point['payload']['deep_link_url']}")
                        print(f"  - text (first 100 chars): {point['payload']['text'][:100]}...")
                        print(f"  - metadata: {point['payload']['metadata']}")
                        print("-" * 70)
                        print()

                total_processed += len(batch_chunks)
                success = True
                print(f"   ✅ Batch {batch_num} completed")

                # Wait between batches (except last)
                if batch_end < len(chunks):
                    print(f"   ⏱️  Waiting {BASE_DELAY}s...")
                    time.sleep(BASE_DELAY)

            except Exception as e:
                error_str = str(e)
                print()
                print(f"   ❌ Embedding Error in Batch {batch_num}")
                print("   " + "-" * 66)
                print(f"   Error Type: {type(e).__name__}")
                print(f"   Error Message: {error_str}")
                print()

                if "429" in error_str or "rate_limit" in error_str.lower():
                    attempt += 1
                    if attempt < MAX_RETRIES:
                        backoff_delay = exponential_backoff(attempt)
                        print(f"   ⚠️  Rate limit detected! Attempt {attempt}/{MAX_RETRIES}")
                        print(f"   ⏱️  Waiting {backoff_delay:.1f}s (exponential backoff)...")
                        print()
                        time.sleep(backoff_delay)
                    else:
                        print(f"   ❌ Max retries reached for batch {batch_num}")
                        print(f"   📊 Processed {total_processed}/{len(chunks)} chunks before failure")
                        print()
                        break
                else:
                    print("   Full Traceback:")
                    print("   " + "-" * 66)
                    for line in traceback.format_exc().split('\n'):
                        print(f"   {line}")
                    print("   " + "-" * 66)
                    print()
                    break

    # Upload to Qdrant
    if all_points:
        print()
        print("=" * 70)
        print("🧪 DIAGNOSTIC TEST 3: Qdrant Upload")
        print("=" * 70)
        print()
        print(f"Attempting to upload {len(all_points)} points to Qdrant...")
        print()

        try:
            qdrant.upsert_points(all_points)
            print(f"✅ Upload SUCCESSFUL: {len(all_points)} chunks from {filename}")
            print()
            return len(all_points)

        except Exception as e:
            print(f"❌ Qdrant Upload FAILED")
            print()
            print("Full Error Details:")
            print("-" * 70)
            print(f"Error Type: {type(e).__name__}")
            print(f"Error Message: {str(e)}")

            # Try to get detailed error content
            if hasattr(e, 'content'):
                print()
                print("Error Content:")
                print(e.content)

            if hasattr(e, 'response'):
                print()
                print("Response Details:")
                print(e.response)

            print()
            print("Full Traceback:")
            print(traceback.format_exc())
            print("-" * 70)
            print()
            return 0
    else:
        print()
        print(f"⚠️  No points to upload from {filename}")
        print(f"   Total processed: {total_processed} chunks")
        print()
        return 0


def main():
    """
    Main diagnostic ingestion process
    """
    print("=" * 70)
    print("RAG Chatbot - DIAGNOSTIC Data Ingestion")
    print("=" * 70)
    print()
    print("🔍 DIAGNOSTIC MODE ENABLED")
    print(f"   • Batch size: {BATCH_SIZE} chunk(s) per API call")
    print(f"   • Chunk size: {CHUNK_SIZE} characters")
    print(f"   • Delay between batches: {BASE_DELAY}s")
    print(f"   • Max retries: {MAX_RETRIES}")
    print()
    print("This mode provides detailed error logging to diagnose issues.")
    print()

    # Target folder for textbook markdown files
    TARGET_FOLDER = "../textbook/textbook/docs"

    # Check if folder exists
    if not os.path.exists(TARGET_FOLDER):
        print(f"❌ Folder not found: {TARGET_FOLDER}")
        print(f"💡 Update TARGET_FOLDER in ingest.py to your docs path")
        return

    print(f"📂 Target folder: {TARGET_FOLDER}")
    print()

    # Initialize services ONCE (not in loop)
    print("🔧 Initializing services...")
    try:
        cohere = CohereService()
        print("✅ Cohere service initialized")

        qdrant = QdrantService()
        print("✅ Qdrant service initialized")
    except Exception as e:
        print(f"❌ Service initialization failed: {e}")
        print(f"💡 Check your .env file has COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY")
        print()
        print("Full Traceback:")
        print(traceback.format_exc())
        return

    print()

    # DIAGNOSTIC TEST 1: Test Cohere API with single word
    if not test_cohere_api(cohere):
        print("⚠️  Cohere API test failed. Cannot proceed with ingestion.")
        print("   Fix the Cohere API issue before continuing.")
        return

    # Create collection ONCE (checks if exists to avoid 409 errors)
    print("=" * 70)
    print("📦 Setting up Qdrant collection...")
    print("=" * 70)
    print()
    try:
        qdrant.create_collection(vector_size=1024)
    except Exception as e:
        print(f"❌ Collection setup failed: {e}")
        print()
        print("Full Traceback:")
        print(traceback.format_exc())
        return

    print()
    print("=" * 70)
    print("Starting file processing...")
    print("=" * 70)
    print()

    # Process all files
    total_files = 0
    total_chunks = 0

    for root, dirs, files in os.walk(TARGET_FOLDER):
        for file in files:
            if file.endswith(".md") or file.endswith(".txt"):
                file_path = os.path.join(root, file)
                total_files += 1

                chunks_uploaded = process_file(file_path, cohere, qdrant)
                total_chunks += chunks_uploaded
                print()  # Blank line between files

    # Summary
    print("=" * 70)
    print("✨ Diagnostic Ingestion Complete!")
    print("=" * 70)
    print(f"📊 Files processed: {total_files}")
    print(f"📊 Total chunks uploaded: {total_chunks}")
    print()

    # Verify collection (FIXED: use points_count)
    try:
        info = qdrant.get_collection_info()
        print(f"✅ Collection '{info['name']}' now has {info['points_count']} points")
    except Exception as e:
        print(f"⚠️  Could not verify collection: {e}")
        print()
        print("Full Traceback:")
        print(traceback.format_exc())


if __name__ == "__main__":
    main()
