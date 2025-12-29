# Diagnostic Mode - Ingestion Script

## Overview

The `ingest.py` script has been updated with comprehensive diagnostic features to help identify the root cause of "0 chunks uploaded" issues before purchasing a production Cohere API key.

## What Changed

### Diagnostic Configuration (Lines 18-22)

```python
# DIAGNOSTIC CONFIGURATION - Minimal batching for testing
BATCH_SIZE = 1  # Process 1 chunk at a time for diagnostics
CHUNK_SIZE = 2000  # Characters per chunk
BASE_DELAY = 20  # 20 seconds between batches for diagnostics
MAX_RETRIES = 5  # Maximum retry attempts for 429 errors
```

**Why minimal batching?**
- Processes 1 chunk at a time to isolate exactly where failures occur
- 20 second delay gives Trial API maximum time to reset between requests
- Makes it clear if the problem is rate limits vs. something else

---

## Diagnostic Tests

### Test 1: Cohere API Connection (Lines 40-80)

**Purpose**: Verify Cohere API is accessible before processing any files

**What it does**:
- Attempts to embed a single word: `"test"`
- If successful: Shows embedding dimension, first 5 values, and type
- If failed: Shows full raw error with traceback

**Expected Output (Success)**:
```
======================================================================
🧪 DIAGNOSTIC TEST 1: Cohere API Connection
======================================================================

Testing Cohere API by embedding single word 'test'...

✅ Cohere API Test PASSED
   • Embedding dimension: 1024
   • First 5 values: [0.123, -0.456, 0.789, ...]
   • Embedding type: <class 'list'>
```

**Expected Output (Failure)**:
```
❌ Cohere API Test FAILED

Full Raw Error from Cohere:
----------------------------------------------------------------------
Error Type: CohereAPIError
Error Message: status_code: 401, body: {'message': 'invalid api token'}

Full Traceback:
[... full stack trace ...]
----------------------------------------------------------------------

💡 This indicates the problem is with Cohere API access, not just rate limits.
   Check your COHERE_API_KEY in .env file.
```

**What This Tells You**:
- **If this passes**: Your Cohere API key is valid and working
- **If this fails with 401**: API key is invalid
- **If this fails with 429**: Even a single word triggers rate limit (very unusual)
- **If this fails with other error**: Network/connectivity issue

---

### Test 2: Point Structure Check (Lines 160-181)

**Purpose**: Verify the data structure sent to Qdrant is correct

**What it does**:
- Prints the first generated Point object in detail
- Shows ID, vector dimension, vector type, and payload structure
- Validates the format matches Qdrant's expectations

**Expected Output**:
```
======================================================================
🧪 DIAGNOSTIC TEST 2: Point Structure Check
======================================================================

First generated Point object:
----------------------------------------------------------------------
ID: 550e8400-e29b-41d4-a716-446655440000
Vector dimension: 1024
Vector type: <class 'list'>
Vector first 3 values: [0.234, -0.567, 0.891]

Payload structure:
  - source: intro.md
  - section_title: intro
  - deep_link_url: /docs/intro
  - text (first 100 chars): # Introduction to Physical AI

Physical AI represents the convergence of artificial intelligen...
  - metadata: {'chunk_index': 0, 'total_chunks': 15}
----------------------------------------------------------------------
```

**What This Tells You**:
- **Vector dimension is 1024**: Correct for Cohere embed-english-v3.0
- **Vector type is list**: Correct format for Qdrant
- **Payload has all fields**: Text, source, metadata all present
- **If dimension is wrong**: Using wrong embedding model
- **If type is wrong**: Data format issue in cohere_service.py

---

### Test 3: Qdrant Upload (Lines 223-263)

**Purpose**: Detailed error reporting for Qdrant upload failures

**What it does**:
- Attempts to upload points to Qdrant
- If failed: Shows error type, message, content, response, and full traceback
- Tries to extract detailed error information from exception object

**Expected Output (Success)**:
```
======================================================================
🧪 DIAGNOSTIC TEST 3: Qdrant Upload
======================================================================

Attempting to upload 5 points to Qdrant...

✅ Upload SUCCESSFUL: 5 chunks from intro.md
```

**Expected Output (Failure)**:
```
❌ Qdrant Upload FAILED

Full Error Details:
----------------------------------------------------------------------
Error Type: UnexpectedResponse
Error Message: Unexpected Response: 400 Bad Request

Error Content:
{'status': {'error': 'Vector dimension mismatch: expected 1024, got 768'}}

Response Details:
<Response [400]>

Full Traceback:
[... full stack trace ...]
----------------------------------------------------------------------
```

**What This Tells You**:
- **400 with dimension mismatch**: Collection created with wrong vector size
- **403 Forbidden**: Authentication issue (API key wrong or cluster paused)
- **500 Internal Server Error**: Qdrant cluster issue
- **Network timeout**: Connectivity problem

---

## Enhanced Error Reporting

### Embedding Errors (Lines 192-221)

Every embedding attempt now shows:
- Error type (e.g., `CohereAPIError`, `RateLimitError`)
- Error message (full text from API)
- Full traceback for debugging

**For 429 Rate Limit Errors**:
```
   ❌ Embedding Error in Batch 2
   ------------------------------------------------------------------
   Error Type: CohereAPIError
   Error Message: status_code: 429, body: {'message': 'rate limit exceeded'}

   ⚠️  Rate limit detected! Attempt 1/5
   ⏱️  Waiting 2.0s (exponential backoff)...
```

**For Other Errors**:
```
   ❌ Embedding Error in Batch 1
   ------------------------------------------------------------------
   Error Type: ValueError
   Error Message: Invalid input text

   Full Traceback:
   ------------------------------------------------------------------
   [... complete stack trace ...]
   ------------------------------------------------------------------
```

---

## How to Use Diagnostic Mode

### Step 1: Run the Script

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate
python ingest.py
```

### Step 2: Read the Diagnostic Output

The script will run three diagnostic tests:

1. **Cohere API Connection Test**: Runs immediately after service initialization
2. **Point Structure Check**: Shows when first point is generated
3. **Qdrant Upload Test**: Shows when first batch uploads

### Step 3: Interpret Results

**Scenario A: Test 1 Fails (Cohere API)**
```
❌ Cohere API Test FAILED
Error Type: CohereAPIError
Error Message: status_code: 401, body: {'message': 'invalid api token'}
```
**Action**: Fix your COHERE_API_KEY in `.env` file

---

**Scenario B: Test 1 Passes, But Embeddings Fail with 429**
```
✅ Cohere API Test PASSED

   ❌ Embedding Error in Batch 1
   Error Type: CohereAPIError
   Error Message: status_code: 429, body: {'message': 'rate limit exceeded'}
```
**Action**: Problem is ONLY rate limits. Consider production key or wait longer between batches.

---

**Scenario C: Tests 1 & 2 Pass, Upload Fails**
```
✅ Cohere API Test PASSED
✅ Point Structure Check shows correct format

❌ Qdrant Upload FAILED
Error Type: UnexpectedResponse
Error Message: 400 Bad Request - Vector dimension mismatch
```
**Action**: Problem is with Qdrant collection setup (wrong vector size)

---

**Scenario D: All Tests Pass, But 0 Chunks Uploaded**
```
✅ Cohere API Test PASSED
✅ Point Structure Check shows correct format
✅ Upload SUCCESSFUL: 5 chunks from intro.md

📊 Total chunks uploaded: 0
```
**Action**: Check if `all_points` list is being cleared somewhere, or if files have no content

---

## Diagnostic Output Summary

### What Gets Logged:

1. **Service Initialization**:
   - Cohere and Qdrant service status
   - Full traceback if initialization fails

2. **Cohere API Test**:
   - Embedding dimension and type
   - Full error details if test fails

3. **File Processing**:
   - Number of chunks per file
   - Batch size and delay settings

4. **Each Batch**:
   - Batch number (X/Y)
   - Success or failure status
   - Full error details if failed
   - Exponential backoff delays for 429 errors

5. **First Point Structure**:
   - ID, vector dimension, vector type
   - Complete payload structure
   - First 100 characters of text

6. **Qdrant Upload**:
   - Number of points being uploaded
   - Success or detailed failure information
   - Error content and response if available

7. **Final Summary**:
   - Total files processed
   - Total chunks uploaded
   - Final collection points count (using `points_count`)

---

## Configuration for Different Tests

### Test 1: Verify Cohere API Works (Current Settings)
```python
BATCH_SIZE = 1
BASE_DELAY = 20
MAX_RETRIES = 5
```
**Goal**: Determine if API key is valid and can generate embeddings

---

### Test 2: Test Faster Processing (If Test 1 Passes)
```python
BATCH_SIZE = 5
BASE_DELAY = 10
MAX_RETRIES = 3
```
**Goal**: See if slightly faster processing still works

---

### Test 3: Production-Like Settings (After Upgrade)
```python
BATCH_SIZE = 20
BASE_DELAY = 3
MAX_RETRIES = 3
```
**Goal**: Maximize throughput with production API key

---

## Fixed Issues

### 1. `points_count` Instead of `vectors_count` (Line 370)

**Before**:
```python
print(f"✅ Collection '{info['name']}' now has {info['vectors_count']} points")
```

**After**:
```python
print(f"✅ Collection '{info['name']}' now has {info['points_count']} points")
```

**Why**: Newer qdrant-client versions use `points_count` attribute

---

### 2. Full Traceback Import (Line 9)

**Added**:
```python
import traceback
```

**Used for**:
- `traceback.format_exc()` to get full stack traces
- Better debugging of unexpected errors

---

### 3. Detailed Qdrant Error Handling (Lines 247-256)

**Added**:
```python
# Try to get detailed error content
if hasattr(e, 'content'):
    print("Error Content:")
    print(e.content)

if hasattr(e, 'response'):
    print("Response Details:")
    print(e.response)
```

**Why**: Qdrant exceptions may have additional detail in `content` or `response` attributes

---

## Next Steps

### If Problem is Rate Limits (429 errors):
1. **Option A**: Wait it out with current Trial key
   - Increase `BASE_DELAY` to 30s or 60s
   - Decrease `BATCH_SIZE` to 1 (already set)
   - Run overnight

2. **Option B**: Upgrade to Production key
   - Much higher rate limits
   - Can use `BATCH_SIZE = 20` and `BASE_DELAY = 3`
   - Will complete in minutes instead of hours

### If Problem is NOT Rate Limits:
- The diagnostic output will show the exact error
- Fix the specific issue (API key, Qdrant config, etc.)
- Re-run the script

### After Diagnostics Complete:
- You'll have clear evidence of whether you need production key
- Or if there's a different issue that needs fixing first

---

## Expected Runtime

With diagnostic settings:
- **Small textbook** (10 files, 50 chunks): ~20 minutes
- **Medium textbook** (50 files, 250 chunks): ~90 minutes
- **Large textbook** (100 files, 500 chunks): ~3 hours

**Why so slow?**
- 1 chunk at a time (BATCH_SIZE = 1)
- 20 second delay between chunks (BASE_DELAY = 20)
- This is intentional for diagnostics

**With production key** (BATCH_SIZE = 20, BASE_DELAY = 3):
- Same large textbook: ~30 minutes

---

## Summary

This diagnostic mode helps you answer:

1. ✅ **Is my Cohere API key valid?** (Test 1)
2. ✅ **Can I generate embeddings at all?** (Test 1)
3. ✅ **Is the point structure correct?** (Test 2)
4. ✅ **Can I upload to Qdrant?** (Test 3)
5. ✅ **Is the problem ONLY rate limits?** (Error analysis)

**Run this diagnostic script once to determine your next action.**

If all tests pass but you're hitting 429 errors, you know the problem is rate limits and a production key will solve it.

If tests fail, the diagnostic output will show exactly what needs to be fixed.
