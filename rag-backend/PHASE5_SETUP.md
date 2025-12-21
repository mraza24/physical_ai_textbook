# Phase 5: Frontend Integration - Setup Guide

## Overview

Phase 5 integrates the RAG backend with the Docusaurus frontend to provide live textbook Q&A functionality.

## Architecture

```
┌─────────────────────┐         ┌──────────────────────┐
│  Docusaurus Frontend│         │   FastAPI Backend    │
│  (React/TypeScript) │────────▶│   (Python)           │
│                     │  HTTP   │                      │
│  - RAGChatbot.tsx   │ POST    │  - /api/query        │
│  - English validate │         │  - /api/ingest       │
│  - Citation display │         │  - /api/health       │
└─────────────────────┘         └──────────────────────┘
         │                               │
         │                               ├────▶ Cohere API (embeddings + chat)
         │                               ├────▶ Qdrant (vector search)
         └───────────────────────────────├────▶ PostgreSQL (query logs)
                                         │
```

## Prerequisites

### Backend Requirements
- Python 3.10+
- Virtual environment (venv)
- PostgreSQL database (Neon Serverless)
- Qdrant Cloud instance
- Cohere API key

### Frontend Requirements
- Node.js 18+ and npm
- Docusaurus 3.x (already installed)

## Setup Instructions

### 1. Backend Setup

#### 1.1 Navigate to Backend Directory
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
```

#### 1.2 Verify Environment Variables
Ensure `.env` file has the following configured:
```bash
# Cohere API
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Neon Serverless PostgreSQL
DATABASE_URL=postgresql://username:password@host/database?sslmode=require

# API Configuration
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001
REQUEST_TIMEOUT_SECONDS=3
SIMILARITY_THRESHOLD=0.7

# Admin API (for ingestion)
ADMIN_API_KEY=your_admin_key_here
```

#### 1.3 Run Database Migrations
```bash
./venv/bin/python scripts/run_migrations.py
```

#### 1.4 Setup Qdrant Collection
```bash
./venv/bin/python scripts/setup_qdrant.py
```

#### 1.5 Ingest Textbook Content (if not done)
```bash
curl -X POST http://localhost:8000/api/ingest \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: your_admin_key_here" \
  -d '{
    "book_path": "../textbook/docs",
    "book_version": "1.0.0",
    "force_reindex": false
  }'
```

#### 1.6 Start Backend Server
```bash
./venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

**Expected Output:**
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [xxxxx] using StatReload
INFO:     Started server process [xxxxx]
INFO:     Waiting for application startup.
🚀 RAG Chatbot API starting up...
   Allowed origins: ['http://localhost:3000', 'http://localhost:3001']
INFO:     Application startup complete.
```

### 2. Frontend Setup

#### 2.1 Navigate to Frontend Directory
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/textbook
```

#### 2.2 Verify Environment Configuration
Ensure `.env.local` file exists with:
```bash
REACT_APP_API_URL=http://localhost:8000
```

#### 2.3 Install Dependencies (if needed)
```bash
npm install
```

#### 2.4 Start Frontend Development Server
```bash
npm start
```

**Expected Output:**
```
> textbook@0.0.0 start
> docusaurus start

[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### 3. Verify End-to-End Integration

#### 3.1 Access Frontend
Open browser to: `http://localhost:3000`

#### 3.2 Test Q&A Flow
1. Scroll to "Ask the Textbook" section
2. Enter a question (e.g., "What is forward kinematics?")
3. Click "Ask" button
4. Verify:
   - Loading state appears ("Thinking...")
   - Answer appears within 3 seconds
   - Citations with deep links are displayed
   - Metadata shows confidence, chunk count, and processing time

#### 3.3 Test Validation
- **English-only**: Try non-English characters → Should show error
- **Empty query**: Submit empty → Should show error
- **Long query**: Enter >500 characters → Should show error
- **Timeout**: Backend should respond <3s or timeout gracefully

#### 3.4 Test Error Handling
Stop backend server:
```bash
# Kill backend process
pkill -f "uvicorn app.main:app"
```

Attempt query → Should show connection error message

Restart backend:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend
./venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

## Component Details

### RAGChatbot Component

**Location:** `textbook/src/components/RAGChatbot/index.tsx`

**Features:**
- Search input with 500 character limit
- English-only validation (client-side)
- Submit button with loading state
- Answer display with formatted text
- Citation list with deep links to textbook sections
- Metadata display (confidence, chunks, processing time)
- Error handling for network, timeout, validation errors
- Responsive design (mobile-friendly)

**API Integration:**
- Endpoint: `POST /api/query`
- Timeout: 5s client-side, 3s server-side
- Request body:
  ```json
  {
    "query_text": "user question here",
    "selected_text": null
  }
  ```
- Response format:
  ```json
  {
    "answer": "Generated answer text",
    "citations": [
      {
        "section_title": "Section Name",
        "deep_link_url": "/docs/chapter#section",
        "chunk_count": 2
      }
    ],
    "confidence": 0.85,
    "retrieved_chunk_count": 5,
    "processing_time_ms": 450
  }
  ```

### Styling

**Location:** `textbook/src/components/RAGChatbot/styles.module.css`

**Theme:**
- Uses Docusaurus CSS variables for theme consistency
- Dark mode support (auto-detects via `--ifm-color-*` variables)
- Academic minimal design
- Responsive breakpoints for mobile

## Troubleshooting

### Backend Issues

**Problem:** Backend won't start
- Check `.env` file exists and has all required variables
- Verify PostgreSQL connection: `psql $DATABASE_URL`
- Verify Qdrant connection: Check URL and API key
- Check Python version: `python --version` (requires 3.10+)

**Problem:** 504 Timeout errors
- Check backend logs for slow operations
- Verify Cohere API key is valid
- Check Qdrant cluster is responsive
- Reduce `top_k` in retriever or increase timeout

**Problem:** 500 Internal Server Error
- Check backend logs: `tail -f logs/app.log` (if configured)
- Verify database migrations ran successfully
- Check Qdrant collection exists: `scripts/setup_qdrant.py`

### Frontend Issues

**Problem:** Frontend won't start
- Check Node.js version: `node --version` (requires 18+)
- Clear node_modules: `rm -rf node_modules && npm install`
- Check port 3000 not in use: `lsof -i :3000`

**Problem:** "Failed to fetch" errors
- Verify backend is running: `curl http://localhost:8000/api/health`
- Check CORS configuration in backend `.env`: `ALLOWED_ORIGINS`
- Check `.env.local` has correct `REACT_APP_API_URL`

**Problem:** Component not rendering
- Check browser console for errors
- Verify import path: `@site/src/components/RAGChatbot`
- Clear Docusaurus cache: `npm run clear`

### Integration Issues

**Problem:** Citations deep links don't work
- Verify textbook content structure matches expected format
- Check `deep_link_url` format in backend chunking logic
- Test deep link manually: `http://localhost:3000/docs/chapter#section`

**Problem:** Answers seem irrelevant
- Check similarity threshold: Lower it in `.env` (e.g., 0.6)
- Verify content was ingested: Check Qdrant collection count
- Test embedding service: Try different queries

## Production Deployment

### Backend Deployment
1. Deploy to cloud provider (e.g., Railway, Render, Fly.io)
2. Update `.env` with production values
3. Set `ALLOWED_ORIGINS` to production frontend URL
4. Configure production PostgreSQL and Qdrant instances
5. Run migrations and setup scripts
6. Ingest production content

### Frontend Deployment
1. Update `.env.local` with production backend URL
2. Build for production: `npm run build`
3. Deploy to hosting (e.g., Vercel, Netlify, GitHub Pages)
4. Verify CORS allows production frontend origin

## Performance Metrics

**Expected Performance:**
- Query response time: <500ms average, <3s max
- Concurrent users: 10+ simultaneous queries
- Success rate: 100% under normal load
- Retrieval accuracy: >70% similarity threshold

**Validated (Phase 4 Load Test):**
- 10 concurrent users: 100% success
- Average response time: 300ms
- All responses <3s requirement

## Next Steps

After Phase 5 completion:
1. Gather user feedback on answer quality
2. Optimize similarity threshold based on usage patterns
3. Add analytics tracking for query patterns
4. Consider streaming responses for longer answers
5. Add session management for conversation history
6. Implement rate limiting for public deployment

## Files Created/Modified

### Frontend Files Created:
- `textbook/src/components/RAGChatbot/index.tsx` (200 lines)
- `textbook/src/components/RAGChatbot/styles.module.css` (180 lines)
- `textbook/.env.local` (4 lines)

### Frontend Files Modified:
- `textbook/src/pages/index.tsx` (integrated RAGChatbot component)

### Documentation:
- `rag-backend/PHASE5_SETUP.md` (this file)

## Support

For issues or questions:
1. Check backend logs for API errors
2. Check browser console for frontend errors
3. Review troubleshooting section above
4. Test components individually (backend health, frontend render)
5. Verify all environment variables are set correctly
