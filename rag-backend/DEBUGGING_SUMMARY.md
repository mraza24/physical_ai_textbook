# RAG Backend HTTP 500 Debugging - Complete Summary

## 📦 What Was Created

I've analyzed your RAG backend and created comprehensive debugging resources:

### 1. **DEBUG_HTTP_500.md** (Main Guide)
   - Complete analysis of all common RAG failure points
   - Step-by-step debugging process
   - Render log checking instructions
   - Dimension mismatch resolution
   - Memory/timeout issue handling

### 2. **debug_backend.sh** (Automated Script)
   - Checks .env file exists
   - Validates API keys are set
   - Tests Cohere API connection
   - Tests Qdrant connection
   - Verifies collection exists and has data
   - Tests embedding generation

### 3. **QUICK_FIX.md** (5-Minute Checklist)
   - Top 5 most common issues (90% of problems)
   - One-command solutions
   - Quick verification steps

---

## 🎯 Most Likely Issues (Priority Order)

### 1. **Render Environment Variables Not Set** (90% probability)

**Symptom**: Works locally, fails on Render

**Root Cause**: Render doesn't load `.env` files automatically

**Fix**:
```
1. Go to: https://dashboard.render.com
2. Your Service → Environment tab
3. Add:
   - COHERE_API_KEY
   - QDRANT_URL
   - QDRANT_API_KEY
4. Manual Deploy → Clear build cache & deploy
```

---

### 2. **Qdrant Collection is Empty** (70% probability)

**Symptom**: Backend runs but returns "no relevant information found"

**Root Cause**: `textbook_chunks` collection has 0 vectors

**Fix**:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate
python ingest.py
```

**Verify**:
```bash
./debug_backend.sh
# Look for: Points count: 150+ (not 0)
```

---

### 3. **Collection Doesn't Exist** (50% probability)

**Symptom**: Error logs show "Collection 'textbook_chunks' not found"

**Root Cause**: Collection never created in Qdrant

**Fix (Option A - Dashboard)**:
```
1. https://cloud.qdrant.io
2. Create Collection
3. Name: textbook_chunks
4. Vector Size: 1024
5. Distance: Cosine
```

**Fix (Option B - Python)**:
```bash
python3 -c "from app.services.qdrant_service import QdrantService; from dotenv import load_dotenv; load_dotenv(); QdrantService().create_collection(vector_size=1024)"
```

---

### 4. **Invalid Cohere API Key** (30% probability)

**Symptom**: "API key validation failed" in logs

**Root Cause**: Key expired, invalid, or not set

**Fix**:
1. Get new key: https://dashboard.cohere.com
2. Update `.env`:
   ```
   COHERE_API_KEY=your-new-key-here
   ```
3. Update Render environment variables

---

### 5. **Embedding Dimension Mismatch** (10% probability)

**Symptom**: "Dimension mismatch" error

**Root Cause**: Collection created with wrong vector size

**Current Configuration**:
- Embedding Model: `embed-multilingual-v3.0`
- Dimension: **1024**
- Collection Vector Size: **Must be 1024**

**Fix**:
1. Delete old collection in Qdrant dashboard
2. Recreate with `vector_size=1024`
3. Re-run `python ingest.py`

---

## 🚀 How to Use These Resources

### Step 1: Run Automated Diagnosis
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
chmod +x debug_backend.sh
./debug_backend.sh
```

**This will tell you**:
- ✅ What's working
- ❌ What's broken
- 💡 Exact fix for each issue

---

### Step 2: Follow Quick Fix Guide
```bash
# Read the quick fix checklist
cat QUICK_FIX.md

# Most likely: Add environment variables to Render
# Then: Run ingestion if collection is empty
```

---

### Step 3: Check Render Logs
```
Render Dashboard → Your Service → Logs

Look for specific error messages:
- "COHERE_API_KEY must be set"
- "Collection not found"
- "List index out of range"
- "Dimension mismatch"
```

---

### Step 4: Test Locally First
```bash
# Activate venv
source venv/bin/activate  # Linux/Mac
# OR: venv\Scripts\activate  # Windows

# Start backend
uvicorn app.main:app --reload --port 8000

# In another terminal:
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"What are humanoid robots?","selected_text":null}'
```

**If works locally but fails on Render** → Environment variables issue

---

## 📊 Your Backend Architecture

### Current Setup:
```
Frontend: Vercel (Docusaurus)
   ↓ HTTPS
Backend: Render (FastAPI)
   ↓ Cohere API (embeddings + generation)
   ↓ Qdrant Cloud (vector search)
   ↓
Data: textbook_chunks collection
```

### Key Components:

**1. Cohere Service** (`app/services/cohere_service.py`)
   - Embedding Model: `embed-multilingual-v3.0` (1024 dims)
   - Generation Model: `command-r-08-2024`
   - Methods:
     - `embed_text()` - Single embedding
     - `embed_batch()` - Batch embeddings
     - `generate_answer()` - LLM response

**2. Qdrant Service** (`app/services/qdrant_service.py`)
   - Collection: `textbook_chunks`
   - Vector Size: 1024
   - Distance: Cosine similarity
   - Methods:
     - `search()` - Semantic search
     - `create_collection()` - Setup
     - `upsert_points()` - Upload vectors

**3. RAG Service** (`app/services/rag_service.py`)
   - Orchestrates Cohere + Qdrant
   - Query flow:
     1. Embed query → Cohere
     2. Search vectors → Qdrant (top_k=5)
     3. Generate answer → Cohere
     4. Return with citations

**4. API Endpoint** (`app/api/query.py`)
   - `POST /api/query` - Main RAG endpoint
   - `GET /api/query/health` - Service health check

---

## 🔧 Configuration Files

### Environment Variables Required:
```bash
# .env file (local development)
COHERE_API_KEY=your-key-here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key-here

# Optional:
DATABASE_URL=postgresql://...
ADMIN_API_KEY=your-admin-key
ALLOWED_ORIGINS=http://localhost:3000,https://yoursite.com
TOP_K_CHUNKS=5
SIMILARITY_THRESHOLD=0.3
TEMPERATURE_MIN=0.3
MAX_RESPONSE_TOKENS=500
```

### Render Environment (Must Set Manually):
```
Dashboard → Environment → Add Environment Variable

Required:
- COHERE_API_KEY
- QDRANT_URL
- QDRANT_API_KEY

Optional:
- ALLOWED_ORIGINS (for CORS)
```

---

## 📝 Common Error Messages & Solutions

| Error in Logs | Cause | Solution |
|---------------|-------|----------|
| `COHERE_API_KEY must be set in environment variables` | Missing env var | Add to Render dashboard |
| `Collection 'textbook_chunks' not found` | No collection | Create in Qdrant dashboard (vector_size=1024) |
| `List index out of range` | Empty collection | Run `python ingest.py` |
| `Embedding dimension mismatch: expected 384, got 1024` | Wrong collection size | Recreate collection with 1024 |
| `ValueError: QDRANT_URL and QDRANT_API_KEY must be set` | Missing Qdrant creds | Add to Render dashboard |
| `429 Too Many Requests` | Cohere rate limit | Wait 60s or upgrade plan |
| `MemoryError` or `Process killed` | Render RAM limit (512MB) | Reduce top_k or upgrade Render plan |
| `Request timeout after 30s` | Slow Qdrant search | Check collection size, reduce top_k |

---

## 🧪 Testing Commands

### 1. Health Check
```bash
curl https://physical-ai-backend-xnwe.onrender.com/api/health
```

Expected:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-27T...",
  "services": {"cohere": true, "qdrant": true}
}
```

### 2. Query Service Health
```bash
curl https://physical-ai-backend-xnwe.onrender.com/api/query/health
```

Expected:
```json
{
  "cohere": true,
  "qdrant": true,
  "overall": true
}
```

### 3. Actual Query
```bash
curl -X POST https://physical-ai-backend-xnwe.onrender.com/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"What are humanoid robots?","selected_text":null}'
```

Expected:
```json
{
  "answer": "Humanoid robots are robots designed to...",
  "citations": [
    {
      "section_title": "intro",
      "deep_link_url": "/docs/intro",
      "chunk_count": 1
    }
  ],
  "confidence": 0.89,
  "retrieved_chunk_count": 5,
  "processing_time_ms": 1234
}
```

---

## 📁 Files Created

```
rag-backend/
├── DEBUG_HTTP_500.md          ← Full debugging guide
├── debug_backend.sh           ← Automated diagnosis script
├── QUICK_FIX.md               ← 5-minute checklist
└── DEBUGGING_SUMMARY.md       ← This file
```

---

## 🎯 Action Plan

**Immediate (Do This First)**:
1. Run `./debug_backend.sh`
2. Fix any issues it reports
3. If collection empty → Run `python ingest.py`
4. Add environment variables to Render dashboard
5. Redeploy on Render

**Verification**:
1. `curl https://physical-ai-backend-xnwe.onrender.com/api/health`
2. Test query from chatbot UI
3. Check for 200 response (not 500)

**If Still Broken**:
1. Check Render logs for exact error
2. Refer to DEBUG_HTTP_500.md for detailed troubleshooting
3. Test locally to isolate issue (local vs Render)

---

## 📞 Support

**Documentation**:
- Main guide: `DEBUG_HTTP_500.md`
- Quick fix: `QUICK_FIX.md`
- This summary: `DEBUGGING_SUMMARY.md`

**External Resources**:
- Cohere Dashboard: https://dashboard.cohere.com
- Qdrant Cloud: https://cloud.qdrant.io
- Render Dashboard: https://dashboard.render.com

**Logs**:
- Local: Terminal output when running `uvicorn app.main:app --reload`
- Render: Dashboard → Your Service → Logs tab

---

**Created**: 2025-12-27
**Backend URL**: https://physical-ai-backend-xnwe.onrender.com
**Status**: Debugging resources ready for use
