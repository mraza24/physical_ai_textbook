# Cohere Trial API Rate Limit Fix - Complete Solution

## Problem Summary

**Issue**: `ingest.py` stuck in loop of `429 rate_limit` errors with Cohere Trial API

**Symptoms**:
- Script hits rate limit after processing only a few chunks
- Even with 20s wait time, continues to fail
- Often results in 0 total chunks uploaded
- Inefficient: processing chunks one-by-one

**Root Cause**:
- Cohere Trial API: ~10 requests per minute limit
- Original approach: 1 chunk = 1 API call
- With hundreds of chunks, rapidly exceeds rate limit
- Simple fixed delay (20s) doesn't adapt to sustained rate limiting

---

## Solution Overview

Implemented **batch processing** with **exponential backoff** to stay within Trial API limits:

1. **Batching**: Group 10 chunks per API call → 10x fewer requests
2. **Exponential Backoff**: Smart retry delays (2s, 4s, 8s, 16s, 32s...)
3. **Base Delay**: 6 seconds between batches to prevent rate limit hits
4. **Max Retries**: 5 attempts per batch before giving up

**Performance Improvement**:
- **Before**: 1 chunk/5s = 12 chunks/min = constant 429 errors
- **After**: 10 chunks/6s = 100 chunks/min within ~10 API calls/min limit

---

## Changes Made to `ingest.py`

### 1. Configuration Constants (Lines 17-21)

```python
# Configuration for rate limit handling
BATCH_SIZE = 10  # Cohere Trial: batch 10 chunks per API call
CHUNK_SIZE = 2000  # Characters per chunk
BASE_DELAY = 6  # Base delay between batches (seconds)
MAX_RETRIES = 5  # Maximum retry attempts for 429 errors
```

**Why These Values**:
- `BATCH_SIZE = 10`: Matches Cohere Trial ~10 requests/minute limit
- `CHUNK_SIZE = 2000`: Good balance for context (already optimized)
- `BASE_DELAY = 6`: Ensures ~10 batches/min (within rate limit)
- `MAX_RETRIES = 5`: Gives enough attempts with backoff (up to 64s total)

---

### 2. Exponential Backoff Function (Lines 24-36)

```python
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
```

**Backoff Schedule**:
- Attempt 0: 2s
- Attempt 1: 4s
- Attempt 2: 8s
- Attempt 3: 16s
- Attempt 4: 32s
- Attempt 5+: 64s, 120s (capped)

---

### 3. Batch Processing in `process_file()` (Lines 39-149)

#### Before (One-by-One Processing):
```python
for chunk in chunks:
    embedding = cohere.embed_text(chunk)  # 1 API call per chunk
    time.sleep(5)  # Fixed delay
```

#### After (Batch Processing):
```python
# Process chunks in batches of 10
for batch_start in range(0, len(chunks), BATCH_SIZE):
    batch_end = min(batch_start + BATCH_SIZE, len(chunks))
    batch_chunks = chunks[batch_start:batch_end]

    print(f"   📦 Batch {batch_num}/{total_batches}: {len(batch_chunks)} chunks...")

    # Retry with exponential backoff
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
            print(f"   ✅ Batch {batch_num} completed")

            # Wait between batches (except last)
            if batch_end < len(chunks):
                print(f"   ⏱️  Waiting {BASE_DELAY}s...")
                time.sleep(BASE_DELAY)

        except Exception as e:
            error_str = str(e)
            if "429" in error_str or "rate_limit" in error_str.lower():
                attempt += 1
                if attempt < MAX_RETRIES:
                    backoff_delay = exponential_backoff(attempt)
                    print(f"   ⚠️  Rate limit! Attempt {attempt}/{MAX_RETRIES}, waiting {backoff_delay:.1f}s...")
                    time.sleep(backoff_delay)
                else:
                    print(f"   ❌ Max retries reached. Processed {total_processed}/{len(chunks)} chunks")
                    break
            else:
                print(f"   ❌ Embedding error: {e}")
                break
```

**Key Improvements**:
- **Batch API Call**: `embed_batch()` processes 10 chunks in one request
- **Progress Tracking**: Shows batch X/Y progress
- **Smart Retry**: Exponential backoff on 429 errors
- **Graceful Degradation**: Reports partial progress if max retries hit

---

### 4. Optimization Info Display (Lines 160-165)

```python
print("⚡ Optimizations:")
print(f"   • Batch processing: {BATCH_SIZE} chunks per API call")
print(f"   • Chunk size: {CHUNK_SIZE} characters")
print(f"   • Exponential backoff for rate limits")
print(f"   • Base delay between batches: {BASE_DELAY}s")
```

**Purpose**: Gives users visibility into rate limit handling strategy

---

## Expected Output

### Successful Run:
```
======================================================================
RAG Chatbot - Textbook Data Ingestion
======================================================================

⚡ Optimizations:
   • Batch processing: 10 chunks per API call
   • Chunk size: 2000 characters
   • Exponential backoff for rate limits
   • Base delay between batches: 6s

📂 Target folder: ../textbook/textbook/docs

🔧 Initializing services...
✅ Cohere service initialized
✅ Qdrant service initialized

📦 Setting up Qdrant collection...
✅ Collection 'textbook_chunks' already exists

======================================================================
Starting file processing...
======================================================================

⏳ Processing 15 chunks from intro.md...
   Using batch size: 10 chunks per request
   📦 Batch 1/2: 10 chunks...
   ✅ Batch 1 completed
   ⏱️  Waiting 6s...
   📦 Batch 2/2: 5 chunks...
   ✅ Batch 2 completed
   📤 Uploading 15 points to Qdrant...
✅ Uploaded 15 chunks from intro.md

======================================================================
✨ Ingestion Complete!
======================================================================
📊 Files processed: 12
📊 Total chunks uploaded: 347

✅ Collection 'textbook_chunks' now has 347 points
```

### With Rate Limit Hit (Handled Gracefully):
```
⏳ Processing 25 chunks from module1.md...
   Using batch size: 10 chunks per request
   📦 Batch 1/3: 10 chunks...
   ✅ Batch 1 completed
   ⏱️  Waiting 6s...
   📦 Batch 2/3: 10 chunks...
   ⚠️  Rate limit! Attempt 1/5, waiting 2.0s...
   ✅ Batch 2 completed
   ⏱️  Waiting 6s...
   📦 Batch 3/3: 5 chunks...
   ✅ Batch 3 completed
   📤 Uploading 25 points to Qdrant...
✅ Uploaded 25 chunks from module1.md
```

---

## Testing the Fix

### 1. Run the Optimized Ingestion

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate
python ingest.py
```

### 2. Monitor Progress

Watch for:
- **Batch progress**: "Batch 1/5: 10 chunks..."
- **Rate limit handling**: "⚠️ Rate limit! Attempt X/5..."
- **Successful uploads**: "✅ Uploaded X chunks from file.md"

### 3. Verify Results

Check final summary:
```
📊 Files processed: X
📊 Total chunks uploaded: Y
✅ Collection 'textbook_chunks' now has Y points
```

---

## Configuration Tuning

If you still hit rate limits frequently, adjust these values in `ingest.py`:

### For More Aggressive Rate Limit Handling:
```python
BATCH_SIZE = 5      # Smaller batches (fewer chunks per call)
BASE_DELAY = 10     # Longer wait between batches
MAX_RETRIES = 8     # More retry attempts
```

### For Faster Processing (if you upgrade from Trial):
```python
BATCH_SIZE = 20     # Larger batches (for paid tier)
BASE_DELAY = 3      # Shorter wait
MAX_RETRIES = 3     # Fewer retries needed
```

---

## Alternative: `ingest_optimized.py`

A standalone optimized version is also available at:
```
/mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend/ingest_optimized.py
```

Both files contain the same optimization logic. Use `ingest_optimized.py` if you want to keep the original `ingest.py` unchanged.

---

## Technical Details

### Why Batch Processing Works

**Cohere Trial API Limits**:
- ~10 requests per minute
- Each request can contain multiple texts (up to 96)

**Original Approach**:
- 100 chunks = 100 API calls = exceeds limit in <10 minutes

**Optimized Approach**:
- 100 chunks ÷ 10 batch size = 10 API calls
- 10 calls × 6s delay = 60 seconds total
- Well within 10 requests/minute limit

### Why Exponential Backoff Works

**Fixed Delay Problem**:
- Waits same time regardless of how long rate limit lasts
- May retry too soon or waste time waiting too long

**Exponential Backoff Solution**:
- Adapts to sustained rate limiting
- Quick initial retries for temporary limits
- Longer waits for persistent limits
- Capped at 120s to prevent infinite waits

---

## Summary

**Files Modified**:
- `ingest.py` - Added batching, exponential backoff, configuration constants

**Key Changes**:
1. Configuration constants for tunable rate limit handling
2. `exponential_backoff()` function for smart retry delays
3. Batch processing in `process_file()` using `embed_batch()`
4. Optimization info display in `main()`

**Expected Results**:
- 10x fewer API calls (batching)
- Graceful handling of 429 errors (exponential backoff)
- Successful ingestion within Trial API limits
- Clear progress tracking and error messages

**Status**: ✅ Complete and ready for testing

**Created**: 2025-12-29
