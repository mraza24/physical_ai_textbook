# RAG Chatbot HTTP 500 Error - Debugging Guide

## 🔍 Problem Analysis

Your chatbot UI works, but queries like "What are humanoid robots?" return **HTTP 500** errors. This means your FastAPI backend on Render is crashing.

---

## 📋 Common RAG Backend Issues (Checklist)

### 1. ✅ Environment Variables Not Set on Render

**Issue**: Render doesn't automatically load `.env` files. API keys must be set in Render dashboard.

**Check**:
1. Go to: https://dashboard.render.com → Your Service → Environment
2. Verify these variables exist:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`

**How to Fix**:
```bash
# On Render dashboard:
Environment → Add Environment Variable

Name: COHERE_API_KEY
Value: <your-cohere-api-key>

Name: QDRANT_URL
Value: https://your-cluster.qdrant.io

Name: QDRANT_API_KEY
Value: <your-qdrant-api-key>
```

**After adding**: Click **"Manual Deploy"** → **"Clear build cache & deploy"**

---

### 2. ✅ Qdrant Collection Doesn't Exist

**Issue**: Your backend tries to search `textbook_chunks` collection, but it doesn't exist in Qdrant.

**Check**:
1. Go to: https://cloud.qdrant.io → Your Cluster → Collections
2. Look for collection named: `textbook_chunks`

**If Missing**:

**Option A - Create via Qdrant Dashboard**:
```
1. Go to Qdrant Cloud Dashboard
2. Click "Create Collection"
3. Name: textbook_chunks
4. Vector Size: 1024 (Cohere embed-multilingual-v3.0)
5. Distance: Cosine
6. Click "Create"
```

**Option B - Create via Python Script**:
```python
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv

load_dotenv()
qdrant = QdrantService()
qdrant.create_collection(vector_size=1024)
print("✅ Collection created!")
```

Run:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
python -c "from app.services.qdrant_service import QdrantService; from dotenv import load_dotenv; load_dotenv(); qdrant = QdrantService(); qdrant.create_collection(vector_size=1024); print('✅ Collection created')"
```

---

### 3. ✅ No Data Ingested into Qdrant (Empty Collection)

**Issue**: Collection exists but has **0 vectors**. Backend searches return empty results, then crashes trying to generate answer.

**Check Collection Size**:
```python
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv

load_dotenv()
qdrant = QdrantService()
info = qdrant.get_collection_info()
print(f"Collection: {info['name']}")
print(f"Points: {info['points_count']}")
print(f"Status: {info['status']}")
```

**Expected Output**:
```
Collection: textbook_chunks
Points: 150+  ← Should have data!
Status: green
```

**If Points = 0**: You need to run data ingestion!

---

### 4. ✅ Ingest Textbook Data into Qdrant

**Your ingestion script exists at**: `rag-backend/ingest.py`

**How to Run Ingestion**:

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend

# Activate virtual environment
source venv/bin/activate  # Linux/Mac
# OR
venv\Scripts\activate  # Windows

# Ensure .env is configured
cat .env  # Verify COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY

# Run ingestion
python ingest.py
```

**Expected Output**:
```
⏳ Processing 15 chunks from intro.md...
✅ Uploaded: intro.md
⏳ Processing 23 chunks from module1.md...
✅ Uploaded: module1.md
...
✨ Mission Successful! All files processed.
```

**Important**: The script processes files from `../textbook/textbook/docs` (line 85). Verify this path exists:

```bash
ls -la ../textbook/textbook/docs/
```

If path is wrong, edit `ingest.py` line 85:
```python
TARGET_FOLDER = "/mnt/d/Q4_hackathon1/physical_ai_textbook/textbook/docs"
```

---

### 5. ✅ Embedding Dimension Mismatch

**Issue**: Query embedding (1024 dims) doesn't match collection vector size (e.g., 384 dims).

**Check Current Setup**:
- **Embedding Model**: `embed-multilingual-v3.0` (line 38, cohere_service.py)
- **Vector Size**: **1024 dimensions**
- **Collection Vector Size**: Must be **1024** (line 18, ingest.py)

**If Mismatch**:
1. Delete old collection in Qdrant dashboard
2. Recreate with `vector_size=1024`
3. Re-run ingestion: `python ingest.py`

**Verify in Code**:
```python
# cohere_service.py line 38
self.embedding_model = "embed-multilingual-v3.0"  # 1024 dims

# ingest.py line 18
qdrant.create_collection(vector_size=1024)  # MUST MATCH
```

---

### 6. ✅ Cohere API Key Invalid or Rate Limited

**Issue**: Cohere API rejects requests (401/403/429 errors).

**Check API Key Validity**:
```python
from app.services.cohere_service import CohereService
from dotenv import load_dotenv

load_dotenv()
cohere = CohereService()
is_valid = cohere.check_api_key()
print(f"API Key Valid: {is_valid}")
```

**If False**:
1. Go to: https://dashboard.cohere.com
2. Copy your API key
3. Update `.env`:
   ```
   COHERE_API_KEY=your-actual-key-here
   ```
4. Update Render environment variables

**Rate Limit Issues**:
- Free tier: **100 API calls/minute**
- If hitting limits, add retry logic (already in `ingest.py` line 67-70)

---

### 7. ✅ Render Memory/Timeout Issues

**Issue**: Render Free Tier has **512 MB RAM**. Large embedding operations can crash.

**Check Render Logs**:
```
Render Dashboard → Your Service → Logs

Look for:
- "MemoryError"
- "Out of memory"
- "Request timeout"
- "Process killed"
```

**Solutions**:

**A. Reduce Chunk Size** (less memory per request):
Edit `rag_service.py` line 31:
```python
search_results = self.qdrant.search(query_vector=query_vector, top_k=3)  # Reduced from 5
```

**B. Upgrade Render Plan**:
- Free: 512 MB RAM
- Starter ($7/mo): 512 MB RAM
- Standard ($25/mo): 2 GB RAM

**C. Optimize Embedding Call** (already optimized):
Your code correctly uses `input_type="search_query"` (line 28, rag_service.py)

---

## 🔧 How to Check Render Logs

### Method 1: Render Dashboard (Recommended)
```
1. Go to: https://dashboard.render.com
2. Click your service: "physical-ai-backend"
3. Click "Logs" tab
4. Look for error messages when you send a query
```

### Method 2: Render CLI
```bash
# Install Render CLI
npm install -g @render-web/cli

# Login
render login

# View logs
render logs <your-service-id>
```

### Method 3: Check Specific Error
```bash
# Send a test query and watch logs
curl -X POST https://physical-ai-backend-xnwe.onrender.com/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"What are humanoid robots?","selected_text":null}'

# Then immediately check Render logs
```

**What to Look For in Logs**:
```
❌ BAD: ValueError: COHERE_API_KEY must be set in environment variables
→ Fix: Add COHERE_API_KEY to Render environment

❌ BAD: qdrant_client.exceptions.UnexpectedResponse: Collection 'textbook_chunks' not found
→ Fix: Create collection and run ingestion

❌ BAD: List index out of range
→ Fix: No data in collection, run ingestion

❌ BAD: Embedding dimension mismatch: expected 384, got 1024
→ Fix: Recreate collection with vector_size=1024

✅ GOOD: INFO: Found 5 results for query (score_threshold=0.3)
✅ GOOD: INFO: Generated answer via command-r-08-2024 (length=250)
```

---

## 🧪 Step-by-Step Debugging Process

### Step 1: Test Health Endpoint
```bash
curl https://physical-ai-backend-xnwe.onrender.com/api/health
```

**Expected**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-27T...",
  "services": {
    "cohere": true,
    "qdrant": true
  }
}
```

**If Fails**: Environment variables missing on Render.

---

### Step 2: Test Query Service Health
```bash
curl https://physical-ai-backend-xnwe.onrender.com/api/query/health
```

**Expected**:
```json
{
  "cohere": true,
  "qdrant": true,
  "overall": true
}
```

**If `cohere: false`**: Invalid COHERE_API_KEY.
**If `qdrant: false`**: Invalid QDRANT_URL or QDRANT_API_KEY.

---

### Step 3: Test Simple Query Locally First
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate

# Start backend locally
uvicorn app.main:app --reload --port 8000
```

In another terminal:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"What are humanoid robots?","selected_text":null}'
```

**If Works Locally but Fails on Render**:
→ Environment variables not set on Render
→ Collection exists locally but not on Qdrant Cloud

---

### Step 4: Verify Qdrant Collection Has Data

**Via Python**:
```python
from qdrant_client import QdrantClient
import os

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

info = client.get_collection("textbook_chunks")
print(f"Points: {info.points_count}")
print(f"Vectors: {info.vectors_count}")

# If 0, run: python ingest.py
```

---

### Step 5: Check Render Build Logs
```
Render Dashboard → Your Service → Events

Look for:
- "Deploy succeeded" ← Good
- "Deploy failed" ← Bad (check build logs)
- "Service crashed" ← Memory issue
```

---

## 🚀 Most Likely Fixes (Ordered by Probability)

### 1. **Missing Environment Variables on Render (90% likely)**
```
Fix: Add COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY to Render Environment
Then: Manual Deploy → Clear build cache & deploy
```

### 2. **Empty Qdrant Collection (70% likely)**
```
Fix: Run python ingest.py locally to upload textbook data
Check: Qdrant dashboard shows points_count > 0
```

### 3. **Collection Doesn't Exist (50% likely)**
```
Fix: Create collection with vector_size=1024 in Qdrant dashboard
Then: Run python ingest.py
```

### 4. **Invalid Cohere API Key (30% likely)**
```
Fix: Get new key from https://dashboard.cohere.com
Update: Render environment variables
```

### 5. **Dimension Mismatch (10% likely)**
```
Fix: Delete old collection, recreate with vector_size=1024
Then: Re-run ingestion
```

---

## 📝 Quick Fix Script (Run This First)

```bash
#!/bin/bash
# Save as: debug_backend.sh

cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate

echo "=== RAG Backend Debug ==="

# 1. Check .env file exists
if [ ! -f .env ]; then
    echo "❌ .env file not found! Copy from .env.example"
    exit 1
fi

# 2. Verify API keys are set
echo "Checking environment variables..."
python3 << EOF
import os
from dotenv import load_dotenv
load_dotenv()

required = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
missing = [v for v in required if not os.getenv(v)]

if missing:
    print(f"❌ Missing: {missing}")
    exit(1)
else:
    print("✅ All environment variables set")
EOF

# 3. Check Cohere API key validity
echo "Testing Cohere API..."
python3 << EOF
from app.services.cohere_service import CohereService
from dotenv import load_dotenv
load_dotenv()

try:
    cohere = CohereService()
    valid = cohere.check_api_key()
    print("✅ Cohere API key valid" if valid else "❌ Cohere API key invalid")
except Exception as e:
    print(f"❌ Cohere error: {e}")
EOF

# 4. Check Qdrant collection
echo "Checking Qdrant collection..."
python3 << EOF
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv
load_dotenv()

try:
    qdrant = QdrantService()
    info = qdrant.get_collection_info()
    print(f"✅ Collection: {info['name']}")
    print(f"   Points: {info['points_count']}")
    if info['points_count'] == 0:
        print("⚠️  WARNING: Collection is empty! Run: python ingest.py")
except Exception as e:
    print(f"❌ Qdrant error: {e}")
    print("💡 Solution: Create collection with vector_size=1024")
EOF

echo "=== Debug Complete ==="
```

**Run**:
```bash
chmod +x debug_backend.sh
./debug_backend.sh
```

---

## 🎯 Final Checklist

Before deploying to Render, verify:

- [ ] `.env` file exists with valid keys
- [ ] Cohere API key tested locally
- [ ] Qdrant collection `textbook_chunks` exists
- [ ] Collection has `vector_size=1024`
- [ ] Ingestion completed: `points_count > 0`
- [ ] Local test: `curl http://localhost:8000/api/query` works
- [ ] Render environment variables set (not just `.env`)
- [ ] Render deployed successfully (check logs)
- [ ] Test Render endpoint: `curl https://physical-ai-backend-xnwe.onrender.com/api/health`

---

## 📞 Still Stuck?

**Check Logs**:
```bash
# Render Dashboard → Logs
# Look for the exact error message
```

**Common Error Messages**:

| Error | Meaning | Fix |
|-------|---------|-----|
| `COHERE_API_KEY must be set` | Missing env var | Add to Render dashboard |
| `Collection 'textbook_chunks' not found` | No collection | Create in Qdrant dashboard |
| `List index out of range` | Empty collection | Run `python ingest.py` |
| `Embedding dimension mismatch` | Wrong vector size | Recreate collection (1024) |
| `429 Too Many Requests` | Rate limit | Wait or upgrade Cohere plan |
| `Out of memory` | Render RAM limit | Reduce `top_k` or upgrade plan |

---

**Created**: 2025-12-27
**Backend URL**: https://physical-ai-backend-xnwe.onrender.com
**Qdrant Collection**: textbook_chunks
**Vector Size**: 1024 (Cohere embed-multilingual-v3.0)
