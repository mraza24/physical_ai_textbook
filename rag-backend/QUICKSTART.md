# RAG Chatbot - Quick Start Guide

## Start the System (2 Commands)

### Terminal 1: Backend
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
./venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

**Expected Output:**
```
🚀 RAG Chatbot API starting up...
   Allowed origins: ['http://localhost:3000', 'http://localhost:3001']
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Terminal 2: Frontend
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/textbook
npm start
```

**Expected Output:**
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/physical_ai_textbook/
```

## Access the Application

**URL:** http://localhost:3000/physical_ai_textbook/

**What You'll See:**
1. Homepage with "Physical AI & Humanoid Robotics" header
2. "Ask the Textbook" section below the hero
3. Search box with "Ask a question about robotics, ROS 2, or AI..." placeholder
4. "Ask" button to submit queries

## Try It Out

### Example Queries
1. "What is forward kinematics?"
2. "Explain robot inverse kinematics"
3. "How does sensor fusion work?"
4. "What is SLAM?"
5. "Describe ROS 2 architecture"

### Expected Behavior
1. Enter question → Click "Ask"
2. Button shows "Thinking..." (loading state)
3. Answer appears within 1-3 seconds
4. Citations show sources with clickable links
5. Metadata displays: Confidence %, Retrieved chunks, Processing time

### Test Validation
- **Empty query:** Shows "Please enter a question."
- **Non-English:** Shows "Please enter your question in English only."
- **Too long (>500 chars):** Shows "Question is too long..."

## Health Checks

### Backend Health
```bash
curl http://localhost:8000/api/health
```
**Expected:** `{"status":"healthy","timestamp":"...","message":"Service is healthy"}`

### Frontend Access
```bash
curl -I http://localhost:3000/physical_ai_textbook/
```
**Expected:** `HTTP/1.1 200 OK`

## Stop the System

### Stop Frontend (Terminal 2)
Press `Ctrl+C`

### Stop Backend (Terminal 1)
Press `Ctrl+C`

Or kill processes:
```bash
pkill -f "uvicorn app.main:app"
pkill -f "docusaurus start"
```

## Troubleshooting

### Backend won't start
- Check `.env` file exists with all required variables
- Verify database connection: `psql $DATABASE_URL`
- Check Cohere API key is valid
- Check Qdrant cluster is accessible

### Frontend won't start
- Check Node.js version: `node --version` (requires 18+)
- Clear cache: `npm run clear`
- Reinstall: `rm -rf node_modules && npm install`

### "Failed to fetch" errors
- Ensure backend is running: `curl http://localhost:8000/api/health`
- Check CORS in backend `.env`: `ALLOWED_ORIGINS=http://localhost:3000`
- Verify frontend `.env.local`: `REACT_APP_API_URL=http://localhost:8000`

### 504 Timeout errors
- Normal for first query (cold start)
- Check backend logs for slow operations
- Verify Cohere API is responsive
- Check Qdrant cluster performance

## Full Documentation

- **Setup Guide:** `PHASE5_SETUP.md` (complete setup, troubleshooting, deployment)
- **Completion Checkpoint:** `PHASE5_CHECKPOINT.md` (features, testing, validation)
- **API Docs:** http://localhost:8000/api/docs (FastAPI Swagger UI)

## System Architecture

```
Browser (localhost:3000)
    ↓
Docusaurus Frontend (React)
    ↓ HTTP POST /api/query
FastAPI Backend (localhost:8000)
    ↓
┌────────────┬──────────┬────────────┐
│ Cohere API │ Qdrant   │ PostgreSQL │
│ (Embed+LLM)│ (Vector) │ (Logs)     │
└────────────┴──────────┴────────────┘
```

## Performance

**Expected:**
- Response time: <500ms average
- Max response: <3s (server timeout)
- Concurrent users: 10+ supported
- Success rate: 100%

**Validated (Phase 4 Load Test):**
- 10 concurrent users: 100% success
- Average: 300ms
- All responses <3s ✅

## Next Steps

1. ✅ Start backend and frontend
2. ✅ Access http://localhost:3000/physical_ai_textbook/
3. ✅ Try example queries
4. ✅ Test validation rules
5. ✅ Click citation links
6. 📊 Review query logs (optional)
7. 🚀 Deploy to production (optional)

---

**Status:** Phase 5 Complete - Ready for User Testing
**Last Updated:** 2025-12-21
