# RAG Chatbot Backend

**Production-ready FastAPI backend for retrieval-augmented generation (RAG) chatbot**

Provides question-answering capabilities for physical AI textbook using semantic search (Qdrant) and LLM generation (Cohere).

---

## 🚀 Quick Start

### Prerequisites

- **Python 3.12+** (or 3.10+)
- **Virtual environment** (venv or conda)
- **API Keys:**
  - Cohere API key ([Get here](https://dashboard.cohere.com))
  - Qdrant Cloud cluster ([Get here](https://cloud.qdrant.io))
  - PostgreSQL database (optional, for logging)

### Installation

```bash
# 1. Create virtual environment
python3 -m venv venv

# 2. Activate virtual environment
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows

# 3. Install dependencies
pip install -r requirements.txt

# 4. Configure environment variables
cp .env.example .env
# Edit .env and add your API keys

# 5. Run the server
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# 6. Access the application
# Root: http://localhost:8000/
# Docs: http://localhost:8000/docs
# Health: http://localhost:8000/api/health
```

### Verify Installation

```bash
# Test health endpoint
curl http://localhost:8000/api/health

# Expected response:
# {
#   "status": "healthy",
#   "timestamp": "2025-12-22T...",
#   "message": "Service is healthy"
# }
```

---

## 📁 Project Structure

```
rag-backend/
├── app/
│   ├── __init__.py                 # App package initialization
│   ├── main.py                     # FastAPI application (START HERE)
│   │
│   ├── api/                        # API endpoints
│   │   ├── __init__.py
│   │   ├── query.py                # POST /api/query (RAG endpoint)
│   │   └── health.py               # GET /api/health (health check)
│   │
│   ├── models/                     # Pydantic models
│   │   ├── __init__.py
│   │   ├── query.py                # QueryRequest, QueryResponse, Citation
│   │   └── health.py               # HealthResponse
│   │
│   ├── services/                   # Business logic layer
│   │   ├── __init__.py
│   │   ├── qdrant_service.py       # Qdrant vector database client
│   │   ├── cohere_service.py       # Cohere API client (embeddings + generation)
│   │   └── rag_service.py          # RAG orchestration service
│   │
│   └── middleware/                 # Request/response middleware
│       ├── __init__.py
│       ├── cors.py                 # CORS configuration
│       └── logging_middleware.py   # Request logging
│
├── tests/                          # Test suite
│   ├── __init__.py
│   ├── test_api.py                 # API endpoint tests
│   └── README.md                   # Testing guide
│
├── .env                            # Environment variables (DO NOT COMMIT)
├── .env.example                    # Environment template (COMMIT THIS)
├── requirements.txt                # Python dependencies
├── README.md                       # This file
├── DEPLOYMENT.md                   # Production deployment guide
└── venv/                           # Virtual environment (DO NOT COMMIT)
```

---

## 🔑 Configuration

### Required Environment Variables

See `.env.example` for complete configuration. Critical variables:

```bash
# External Services (REQUIRED)
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Admin Access (REQUIRED for ingestion)
ADMIN_API_KEY=your_secure_admin_key_here

# CORS (REQUIRED for frontend)
ALLOWED_ORIGINS=http://localhost:3000,https://your-app.vercel.app
```

### Getting API Keys

1. **Cohere API Key:**
   - Visit: https://dashboard.cohere.com
   - Sign up for free trial or paid plan
   - Create API key
   - Add to `.env`: `COHERE_API_KEY=...`

2. **Qdrant Cloud:**
   - Visit: https://cloud.qdrant.io
   - Create free cluster
   - Copy cluster URL and API key
   - Add to `.env`: `QDRANT_URL=...` and `QDRANT_API_KEY=...`

3. **Admin API Key:**
   ```bash
   # Generate secure random key
   openssl rand -hex 32
   # Or: python3 -c "import secrets; print(secrets.token_hex(32))"
   # Add to .env: ADMIN_API_KEY=...
   ```

---

## 🌐 API Endpoints

### Root Endpoints

#### `GET /`
Root endpoint to verify backend is running

**Response:**
```json
{
  "message": "RAG Chatbot backend is running",
  "version": "1.0.0",
  "docs": "/docs"
}
```

**Example:**
```bash
curl http://localhost:8000/
```

---

### Health Check

#### `GET /api/health`
Health check endpoint for monitoring

**Response:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-22T10:30:00Z",
  "message": "Service is healthy"
}
```

**Example:**
```bash
curl http://localhost:8000/api/health
```

---

### Query Endpoint

#### `POST /api/query`
Query the RAG chatbot with a question

**Request Body:**
```json
{
  "query_text": "What is forward kinematics?",
  "selected_text": null
}
```

**Response:**
```json
{
  "answer": "Forward kinematics is the process of computing...",
  "citations": [
    {
      "section_title": "Module 1: Robot Kinematics",
      "deep_link_url": "/docs/module1/kinematics#forward-kinematics",
      "chunk_count": 3
    }
  ],
  "confidence": 0.87,
  "retrieved_chunk_count": 5,
  "processing_time_ms": 342
}
```

**Example:**
```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is robotics?",
    "selected_text": null
  }'
```

**Validation Rules:**
- `query_text`: Required, 1-500 characters, English only
- `selected_text`: Optional (not used in v1)

**Error Responses:**
- `400`: Validation error (empty query, non-English, too long)
- `500`: Internal server error (Qdrant/Cohere failure)

---

### API Documentation

#### `GET /docs`
Swagger UI (interactive API documentation)

**Access:** http://localhost:8000/docs

#### `GET /redoc`
ReDoc (alternative API documentation)

**Access:** http://localhost:8000/redoc

#### `GET /openapi.json`
OpenAPI schema (JSON format)

**Access:** http://localhost:8000/openapi.json

---

## 🧪 Testing

### Run Tests

```bash
# Install test dependencies
pip install pytest pytest-asyncio httpx

# Run all tests
pytest tests/ -v

# Run specific test file
pytest tests/test_api.py -v

# Run with coverage
pytest tests/ --cov=app
pytest tests/ --cov=app --cov-report=html  # HTML report
```

### Test Structure

- **Unit Tests:** No external services required
  - Root endpoint tests
  - Health check tests
  - API schema tests

- **Integration Tests:** Require Qdrant + Cohere setup
  - End-to-end query tests
  - Query service health tests

**Note:** Integration tests are skipped by default. See `tests/README.md` for details.

---

## 🏗️ Architecture

### RAG Pipeline

```
User Query
    ↓
[1] Text Embedding (Cohere)
    ↓
[2] Vector Search (Qdrant)
    ↓
[3] Chunk Retrieval (Top 5)
    ↓
[4] Answer Generation (Cohere)
    ↓
Response with Citations
```

### Components

1. **API Layer** (`app/api/`)
   - Handles HTTP requests
   - Input validation (Pydantic)
   - Error handling

2. **Service Layer** (`app/services/`)
   - **QdrantService:** Vector search operations
   - **CohereService:** Embeddings and generation
   - **RAGService:** Orchestrates retrieval + generation

3. **Model Layer** (`app/models/`)
   - Pydantic models for validation
   - Type-safe request/response schemas

4. **Middleware** (`app/middleware/`)
   - CORS configuration
   - Request logging
   - (Future: Rate limiting, auth)

---

## 🔧 Development

### Code Style

```bash
# Install dev dependencies
pip install black flake8 mypy

# Format code
black app/ tests/

# Lint code
flake8 app/ tests/

# Type checking
mypy app/
```

### Hot Reload

```bash
# Run with auto-reload on code changes
uvicorn app.main:app --reload
```

### Debugging

```bash
# Run with debug logging
LOG_LEVEL=DEBUG uvicorn app.main:app --reload

# Or use VS Code debugger:
# 1. Set breakpoint in code
# 2. F5 to start debugging
# 3. Debug Console shows output
```

---

## 🚢 Production Deployment

See **[DEPLOYMENT.md](./DEPLOYMENT.md)** for complete production deployment guide.

### Deployment Platforms

- **Render.com** (Recommended) - Free tier, easy deployment
- **Railway.app** (Alternative) - CLI-first, automatic HTTPS

### Quick Deploy

```bash
# Option 1: Render.com
./deploy-render.sh

# Option 2: Railway.app
./deploy-railway.sh

# Configure Vercel frontend
./configure-vercel.sh https://your-backend-url.com
```

### Production Checklist

- [ ] Deploy backend to cloud (Render/Railway)
- [ ] Configure environment variables in dashboard
- [ ] Set `ALLOWED_ORIGINS` to include frontend URL
- [ ] Deploy frontend to Vercel
- [ ] Configure `REACT_APP_API_URL` in Vercel
- [ ] Test end-to-end integration
- [ ] Run book ingestion
- [ ] Set up monitoring/alerting

---

## 🔒 Security

### Best Practices

1. **Never commit secrets**
   - `.env` is in `.gitignore`
   - Use environment variables in production

2. **Rotate API keys regularly**
   - Quarterly for admin keys
   - Immediately if compromised

3. **Use HTTPS only in production**
   - Automatic on Render/Vercel
   - Never use HTTP for APIs

4. **Limit CORS origins**
   - Only include trusted frontend URLs
   - Don't use wildcard (`*`) in production

5. **Monitor API usage**
   - Check Cohere dashboard for usage
   - Set up rate limiting (future)

---

## 📊 Monitoring

### Health Checks

```bash
# Backend health
curl https://your-backend.com/api/health

# Query service health (Qdrant + Cohere)
curl https://your-backend.com/api/query/health
```

### Logs

**Local development:**
- Logs print to console (stdout)
- INFO level by default

**Production (Render):**
- Dashboard → Logs
- Real-time log streaming

**Production (Railway):**
```bash
railway logs --tail
```

### Metrics

**Performance metrics:**
- Response time: `X-Process-Time` header
- Processing time: `processing_time_ms` in response

**Resource usage:**
- Render Dashboard → Metrics
- Railway Dashboard → Metrics

---

## 🐛 Troubleshooting

### Issue: "Missing environment variables"

**Symptom:** Error on startup about missing `COHERE_API_KEY` or `QDRANT_URL`

**Fix:**
1. Check `.env` file exists: `ls -la .env`
2. Verify variables are set: `cat .env | grep COHERE`
3. Ensure no typos in variable names
4. Restart server after editing `.env`

---

### Issue: "Failed to connect to Qdrant"

**Symptom:** Error when calling `/api/query`

**Fix:**
1. Verify Qdrant URL is correct (includes `https://`)
2. Check Qdrant API key is valid
3. Test Qdrant connection:
   ```bash
   curl $QDRANT_URL/collections \
     -H "api-key: $QDRANT_API_KEY"
   ```
4. Ensure Qdrant cluster is running (check dashboard)

---

### Issue: "CORS error" in browser

**Symptom:** Browser console shows CORS error when frontend calls backend

**Fix:**
1. Add frontend URL to `ALLOWED_ORIGINS` in `.env`
2. Restart backend server
3. Verify CORS headers:
   ```bash
   curl -I http://localhost:8000/api/health \
     -H "Origin: http://localhost:3000"
   ```
4. Check for `Access-Control-Allow-Origin` header

---

### Issue: "Query returns empty answer"

**Symptom:** `/api/query` returns "I cannot find sufficient information..."

**Fix:**
1. Check if Qdrant collection has data:
   ```bash
   curl https://your-backend.com/api/query/health
   ```
2. Verify `textbook_chunks` collection exists
3. Run book ingestion if collection is empty
4. Lower `SIMILARITY_THRESHOLD` in `.env` (try 0.6)

---

## 📚 Resources

### Documentation

- [FastAPI Documentation](https://fastapi.tiangolo.com)
- [Qdrant Documentation](https://qdrant.tech/documentation)
- [Cohere API Documentation](https://docs.cohere.com)
- [Pydantic Documentation](https://docs.pydantic.dev)

### Tools

- [Cohere Dashboard](https://dashboard.cohere.com) - API usage and keys
- [Qdrant Cloud Dashboard](https://cloud.qdrant.io) - Vector database management
- [Render Dashboard](https://dashboard.render.com) - Production deployment
- [Railway Dashboard](https://railway.app) - Alternative deployment

### Community

- [FastAPI Discord](https://discord.gg/fastapi)
- [Qdrant Discord](https://discord.gg/qdrant)

---

## 📝 License

This project is part of the Physical AI Textbook initiative.

---

## 👥 Contributing

### Development Workflow

1. Fork repository
2. Create feature branch: `git checkout -b feature/my-feature`
3. Make changes
4. Run tests: `pytest tests/ -v`
5. Format code: `black app/ tests/`
6. Commit: `git commit -m "Add my feature"`
7. Push: `git push origin feature/my-feature`
8. Create Pull Request

### Code Standards

- **Python style:** Black formatter (line length: 88)
- **Type hints:** Required for all functions
- **Docstrings:** Required for public APIs
- **Tests:** Required for new features
- **Commit messages:** Clear, concise, imperative

---

## 🎯 Roadmap

### v1.0 (Current)
- ✅ FastAPI backend
- ✅ Qdrant vector search
- ✅ Cohere embeddings + generation
- ✅ CORS support
- ✅ Health checks
- ✅ Production deployment

### v1.1 (Planned)
- [ ] Rate limiting
- [ ] Caching (Redis)
- [ ] Async database logging
- [ ] Prometheus metrics
- [ ] Sentry error tracking

### v2.0 (Future)
- [ ] User authentication
- [ ] Query history/sessions
- [ ] Answer streaming (SSE)
- [ ] Multi-language support
- [ ] A/B testing framework

---

## ❓ FAQ

### Q: Can I use a different LLM (e.g., OpenAI)?

**A:** Yes! Replace `CohereService` with your preferred LLM client. Update `app/services/cohere_service.py` and adjust environment variables.

### Q: Can I use a different vector database?

**A:** Yes! Replace `QdrantService` with your preferred vector DB (Pinecone, Weaviate, etc.). Update `app/services/qdrant_service.py`.

### Q: How do I add authentication?

**A:** Add auth middleware in `app/middleware/` and protect endpoints with `Depends()`. See FastAPI security docs.

### Q: How do I scale beyond free tier?

**A:** Upgrade to paid tiers on Render/Qdrant/Cohere, or deploy to AWS/GCP with auto-scaling. See DEPLOYMENT.md.

---

## 📞 Support

For issues, questions, or feedback:

1. Check this README
2. Check [DEPLOYMENT.md](./DEPLOYMENT.md)
3. Check [tests/README.md](./tests/README.md)
4. Open GitHub issue with:
   - Error message
   - Steps to reproduce
   - Environment (Python version, OS)
   - `.env` configuration (without secrets!)

---

**Built with ❤️ using FastAPI, Qdrant, and Cohere**
