"""
Optimized Textbook Data Ingestion Script
Handles Cohere Trial API rate limits with batching and exponential backoff
"""

import uuid
import os
import time
from typing import List, Dict, Any
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration for rate limit handling
BATCH_SIZE = 10  # Cohere Trial allows ~10 requests/min, so batch 10 chunks at once
CHUNK_SIZE = 2000  # Characters per chunk (good balance for context)
BASE_DELAY = 6  # Base delay between batches (seconds)
MAX_RETRIES = 5  # Maximum retry attempts for 429 errors


def exponential_backoff(attempt: int, base_delay: float = 2.0) -> float:
    """
    Calculate exponential backoff delay

    Args:
        attempt: Current attempt number (0-indexed)
        base_delay: Base delay in seconds

    Returns:
        Delay in seconds with exponential backoff
    """
    # Exponential backoff: 2^attempt * base_delay
    # Attempt 0: 2s, Attempt 1: 4s, Attempt 2: 8s, Attempt 3: 16s, etc.
    delay = (2 ** attempt) * base_delay
    max_delay = 120  # Cap at 2 minutes
    return min(delay, max_delay)


def process_file_batched(
    file_path: str,
    cohere: CohereService,
    qdrant: QdrantService
) -> int:
    """
    Process a single file with batched embedding generation

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
    chunks = [
        text_content[i:i+CHUNK_SIZE]
        for i in range(0, len(text_content), CHUNK_SIZE)
    ]

    # Filter out empty chunks
    chunks = [c for c in chunks if c.strip()]

    filename = os.path.basename(file_path)
    print(f"⏳ Processing {len(chunks)} chunks from {filename}...")
    print(f"   Using batch size: {BATCH_SIZE} chunks per request")

    all_points = []
    total_processed = 0

    # Process chunks in batches
    for batch_start in range(0, len(chunks), BATCH_SIZE):
        batch_end = min(batch_start + BATCH_SIZE, len(chunks))
        batch_chunks = chunks[batch_start:batch_end]
        batch_num = (batch_start // BATCH_SIZE) + 1
        total_batches = (len(chunks) + BATCH_SIZE - 1) // BATCH_SIZE

        print(f"   📦 Batch {batch_num}/{total_batches}: Processing {len(batch_chunks)} chunks...")

        # Retry logic with exponential backoff
        success = False
        attempt = 0

        while not success and attempt < MAX_RETRIES:
            try:
                # Generate embeddings for entire batch at once
                embeddings = cohere.embed_batch(
                    texts=batch_chunks,
                    batch_size=len(batch_chunks)  # Process all at once
                )

                # Create points for this batch
                for i, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings)):
                    chunk_index = batch_start + i
                    all_points.append({
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
                    })

                total_processed += len(batch_chunks)
                success = True

                print(f"   ✅ Batch {batch_num} completed ({len(batch_chunks)} chunks embedded)")

                # Rate limiting: wait between batches (except for last batch)
                if batch_end < len(chunks):
                    print(f"   ⏱️  Waiting {BASE_DELAY}s before next batch...")
                    time.sleep(BASE_DELAY)

            except Exception as e:
                error_str = str(e)

                if "429" in error_str or "rate_limit" in error_str.lower():
                    attempt += 1
                    if attempt < MAX_RETRIES:
                        backoff_delay = exponential_backoff(attempt)
                        print(f"   ⚠️  Rate limit hit! Attempt {attempt}/{MAX_RETRIES}")
                        print(f"   ⏱️  Waiting {backoff_delay:.1f}s (exponential backoff)...")
                        time.sleep(backoff_delay)
                    else:
                        print(f"   ❌ Max retries reached for batch {batch_num}")
                        print(f"   💡 Processed {total_processed}/{len(chunks)} chunks before failure")
                        break
                else:
                    print(f"   ❌ Embedding error for batch {batch_num}: {e}")
                    break

    # Upload all points to Qdrant
    if all_points:
        try:
            print(f"   📤 Uploading {len(all_points)} points to Qdrant...")
            qdrant.upsert_points(all_points)
            print(f"✅ Successfully uploaded {len(all_points)} chunks from {filename}")
            return len(all_points)
        except Exception as e:
            print(f"❌ Qdrant upload failed for {filename}: {e}")
            return 0
    else:
        print(f"⚠️  No chunks to upload from {filename}")
        return 0


def main():
    """
    Main ingestion process with optimized batching
    """
    print("=" * 70)
    print("RAG Chatbot - Optimized Textbook Data Ingestion")
    print("=" * 70)
    print()
    print("⚡ Optimizations:")
    print(f"   • Batch processing: {BATCH_SIZE} chunks per API call")
    print(f"   • Chunk size: {CHUNK_SIZE} characters")
    print(f"   • Exponential backoff for rate limits")
    print(f"   • Base delay between batches: {BASE_DELAY}s")
    print()

    # Target folder for textbook markdown files
    TARGET_FOLDER = "../textbook/textbook/docs"

    # Check if folder exists
    if not os.path.exists(TARGET_FOLDER):
        print(f"❌ Folder not found: {TARGET_FOLDER}")
        print(f"💡 Update TARGET_FOLDER in ingest_optimized.py to your docs path")
        return

    print(f"📂 Target folder: {TARGET_FOLDER}")
    print()

    # Initialize services ONCE
    print("🔧 Initializing services...")
    try:
        cohere = CohereService()
        print("✅ Cohere service initialized")

        qdrant = QdrantService()
        print("✅ Qdrant service initialized")
    except Exception as e:
        print(f"❌ Service initialization failed: {e}")
        print(f"💡 Check your .env file has COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY")
        return

    # Create collection ONCE
    print()
    try:
        qdrant.create_collection(vector_size=1024)
    except Exception as e:
        print(f"❌ Collection setup failed: {e}")
        return

    print()
    print("=" * 70)
    print("Starting file processing...")
    print("=" * 70)
    print()

    # Process all files
    total_files = 0
    total_chunks = 0
    start_time = time.time()

    for root, dirs, files in os.walk(TARGET_FOLDER):
        for file in files:
            if file.endswith(".md") or file.endswith(".txt"):
                file_path = os.path.join(root, file)
                total_files += 1

                chunks_uploaded = process_file_batched(file_path, cohere, qdrant)
                total_chunks += chunks_uploaded
                print()  # Blank line between files

    # Summary
    elapsed_time = time.time() - start_time
    print("=" * 70)
    print("✨ Ingestion Complete!")
    print("=" * 70)
    print(f"📊 Files processed: {total_files}")
    print(f"📊 Total chunks uploaded: {total_chunks}")
    print(f"⏱️  Total time: {elapsed_time:.1f} seconds")
    if total_chunks > 0:
        print(f"📈 Average: {elapsed_time/total_chunks:.2f}s per chunk")
    print()

    # Verify collection
    try:
        info = qdrant.get_collection_info()
        print(f"✅ Collection '{info['name']}' now has {info['points_count']} points")
    except Exception as e:
        print(f"⚠️  Could not verify collection: {e}")


if __name__ == "__main__":
    main()
