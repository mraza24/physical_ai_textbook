# HTTP 500 Quick Fix - 5-Minute Checklist

## 🚨 Problem
Your RAG Chatbot UI works, but queries return **HTTP 500** errors from the Render backend.

---

## ⚡ Quick Diagnosis (Run This First)

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
chmod +x debug_backend.sh
./debug_backend.sh
```

**This script checks**:
1. ✅ .env file exists
2. ✅ API keys are set (not placeholders)
3. ✅ Cohere API is reachable
4. ✅ Qdrant connection works
5. ✅ Collection exists and has data
6. ✅ Embeddings generate correctly

---

## 🎯 Most Common Fixes (90% of Issues)

### Fix #1: Render Environment Variables Not Set (★★★★★)

**Problem**: Render doesn't use `.env` files automatically.

**Solution**:
1. Go to: https://dashboard.render.com
2. Click your service: **physical-ai-backend-xnwe**
3. Click **Environment** tab
4. Add these variables:

```
COHERE_API_KEY = <your-cohere-api-key>
QDRANT_URL = https://your-cluster.qdrant.io
QDRANT_API_KEY = <your-qdrant-api-key>
```

5. Click **"Manual Deploy"** → **"Clear build cache & deploy"**
6. Wait 2-3 minutes for deployment
7. Test: `curl https://physical-ai-backend-xnwe.onrender.com/api/health`

---

### Fix #2: Qdrant Collection is Empty (★★★★☆)

**Problem**: Collection exists but has 0 vectors.

**Check**:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate  # Linux/Mac
# OR: venv\Scripts\activate  # Windows

python3 << EOF
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv
load_dotenv()
info = QdrantService().get_collection_info()
print(f"Points: {info['points_count']}")
EOF
```

**If Points = 0**, run ingestion:
```bash
python ingest.py
```

Expected output:
```
⏳ Processing 15 chunks from intro.md...
✅ Uploaded: intro.md
⏳ Processing 23 chunks from module1.md...
✅ Uploaded: module1.md
...
✨ Mission Successful! All files processed.
```

**Verify**:
```bash
python3 << EOF
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv
load_dotenv()
info = QdrantService().get_collection_info()
print(f"Points: {info['points_count']}")  # Should be > 0 now
EOF
```

---

### Fix #3: Collection Doesn't Exist (★★★☆☆)

**Problem**: `textbook_chunks` collection not created in Qdrant.

**Solution**:

**Option A - Via Qdrant Dashboard**:
1. Go to: https://cloud.qdrant.io
2. Login to your cluster
3. Click **"Create Collection"**
4. Name: `textbook_chunks`
5. Vector Size: `1024`
6. Distance: `Cosine`
7. Click **Create**

**Option B - Via Python**:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate

python3 << EOF
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv
load_dotenv()
qdrant = QdrantService()
qdrant.create_collection(vector_size=1024)
print("✅ Collection created!")
EOF
```

Then run ingestion:
```bash
python ingest.py
```

---

### Fix #4: Wrong Ingestion Path (★★★☆☆)

**Problem**: `ingest.py` looks for files in wrong directory.

**Check Current Path** (line 85 of `ingest.py`):
```python
TARGET_FOLDER = "../textbook/textbook/docs"
```

**Verify Path Exists**:
```bash
ls -la ../textbook/textbook/docs/
```

**If Path Wrong**, edit `ingest.py`:
```python
# Line 85 - Update to your actual docs path:
TARGET_FOLDER = "/mnt/d/Q4_hackathon1/physical_ai_textbook/textbook/docs"
```

Then run:
```bash
python ingest.py
```

---

### Fix #5: Invalid Cohere API Key (★★☆☆☆)

**Problem**: API key expired or invalid.

**Test**:
```bash
python3 << EOF
from app.services.cohere_service import CohereService
from dotenv import load_dotenv
load_dotenv()
is_valid = CohereService().check_api_key()
print(f"Valid: {is_valid}")
EOF
```

**If False**:
1. Go to: https://dashboard.cohere.com
2. Login and copy your API key
3. Update `.env`:
   ```
   COHERE_API_KEY=your-new-key-here
   ```
4. Update Render environment variables
5. Redeploy on Render

---

## 📊 Check Render Logs

**Where to Look**:
```
https://dashboard.render.com
→ Your Service: physical-ai-backend-xnwe
→ Logs tab
```

**Common Error Messages**:

| Log Message | Meaning | Fix |
|-------------|---------|-----|
| `COHERE_API_KEY must be set` | Missing env var | Add to Render dashboard |
| `Collection 'textbook_chunks' not found` | No collection | Create collection (see Fix #3) |
| `List index out of range` | Empty collection | Run `python ingest.py` |
| `Embedding dimension mismatch` | Wrong vector size | Recreate collection with 1024 |
| `429 Too Many Requests` | Rate limit | Wait 1 min or upgrade plan |
| `Out of memory` | RAM limit (512MB) | Reduce top_k or upgrade Render |

---

## 🧪 Test Your Backend Locally

**Start Backend**:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
source venv/bin/activate
uvicorn app.main:app --reload --port 8000
```

**Test Query**:
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"What are humanoid robots?","selected_text":null}'
```

**Expected Response**:
```json
{
  "answer": "Humanoid robots are...",
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

**If Works Locally but Fails on Render**:
→ Environment variables not set on Render (see Fix #1)

---

## ✅ Final Verification Checklist

Before marking as "fixed", verify:

- [ ] `./debug_backend.sh` runs without errors
- [ ] Qdrant collection has `points_count > 0`
- [ ] Local query works: `curl http://localhost:8000/api/query`
- [ ] Render environment variables set (not just `.env`)
- [ ] Render health check works: `curl https://physical-ai-backend-xnwe.onrender.com/api/health`
- [ ] Frontend chatbot receives 200 response (not 500)

---

## 📞 Still Getting HTTP 500?

**Advanced Debugging**:

1. **Check Exact Error in Render Logs**:
   ```
   Render Dashboard → Logs
   Send a query from chatbot UI
   Look for Python traceback
   ```

2. **Test Render Endpoints Directly**:
   ```bash
   # Health check
   curl https://physical-ai-backend-xnwe.onrender.com/api/health

   # Query service health
   curl https://physical-ai-backend-xnwe.onrender.com/api/query/health

   # Actual query
   curl -X POST https://physical-ai-backend-xnwe.onrender.com/api/query \
     -H "Content-Type: application/json" \
     -d '{"query_text":"test","selected_text":null}'
   ```

3. **Check Dimension Mismatch**:
   ```python
   # Verify embedding model dimension
   from app.services.cohere_service import CohereService
   from dotenv import load_dotenv
   load_dotenv()

   cohere = CohereService()
   test_vec = cohere.embed_text("test", input_type="search_query")
   print(f"Dimension: {len(test_vec)}")  # Should be 1024
   ```

4. **Verify Collection Vector Size**:
   - Go to Qdrant dashboard
   - Check collection settings
   - Vector size MUST be 1024 (to match Cohere embed-multilingual-v3.0)

---

**For Full Debugging Guide**: See `DEBUG_HTTP_500.md`

**Created**: 2025-12-27
