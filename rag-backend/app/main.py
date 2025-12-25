"""
RAG Chatbot Backend - FastAPI Application
Production-ready backend for retrieval-augmented generation chatbot

HOW TO RUN:
===========

1. Create virtual environment:
   python3 -m venv venv

2. Activate virtual environment:
   source venv/bin/activate  # On Linux/Mac
   venv\\Scripts\\activate     # On Windows

3. Install dependencies:
   pip install -r requirements.txt

4. Configure environment variables:
   - Copy .env.example to .env
   - Update COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL
   - Adjust other settings as needed

5. Run the server:
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

6. Access the application:
   - Root endpoint: http://localhost:8000/
   - API docs: http://localhost:8000/docs
   - Health check: http://localhost:8000/api/health
   - Query endpoint: POST http://localhost:8000/api/query

7. Test with curl:
   curl -X POST http://localhost:8000/api/query \\
     -H "Content-Type: application/json" \\
     -d '{"query_text":"What is robotics?","selected_text":null}'

IMPORTANT:
==========
⚠️ Before running, ensure you have:
   1. Valid Cohere API key (get from https://dashboard.cohere.com)
   2. Qdrant Cloud cluster URL and API key
   3. PostgreSQL database URL (optional for logging)
   4. All credentials added to .env file

⚠️ For production deployment:
   - See DEPLOYMENT.md for cloud deployment instructions
   - Update ALLOWED_ORIGINS to include your frontend URL
   - Use proper secrets management (not .env files)
   - Set up monitoring and logging

Compatible with: Python 3.12+
"""

import logging
import os
from contextlib import asynccontextmanager

from fastapi import FastAPI
from dotenv import load_dotenv

# Import middleware
from app.middleware import setup_cors, setup_logging
from app.middleware.logging_middleware import log_requests

# Import API routers
from app.api import query_router, health_router

# Load environment variables from .env file
# ⚠️ USER TODO: Create .env file with your credentials (copy from .env.example)
load_dotenv()

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for startup and shutdown events

    Startup:
        - Configure logging
        - Validate environment variables
        - Log startup message

    Shutdown:
        - Log shutdown message
        - Clean up resources (if needed)
    """
    # Startup
    setup_logging()
    logger.info("=" * 60)
    logger.info("RAG Chatbot Backend Starting...")
    logger.info("=" * 60)

    # Validate critical environment variables
    required_vars = ["COHERE_API_KEY", "QDRANT_URL", "QDRANT_API_KEY"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        logger.error(f"Missing required environment variables: {missing_vars}")
        logger.error("⚠️  Please check your .env file and ensure all credentials are set")
        logger.error("⚠️  See .env.example for required variables")
    else:
        logger.info("✓ All required environment variables found")

    logger.info(f"✓ CORS allowed origins: {os.getenv('ALLOWED_ORIGINS', 'http://localhost:3000')}")
    logger.info(f"✓ Top K chunks: {os.getenv('TOP_K_CHUNKS', '5')}")
    logger.info(f"✓ Similarity threshold: {os.getenv('SIMILARITY_THRESHOLD', '0.7')}")
    logger.info("=" * 60)
    logger.info("Backend is ready! Access API docs at /docs")
    logger.info("=" * 60)

    yield  # Application runs here

    # Shutdown
    logger.info("=" * 60)
    logger.info("RAG Chatbot Backend Shutting Down...")
    logger.info("=" * 60)


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot Backend",
    description=(
        "Production-ready backend for retrieval-augmented generation chatbot. "
        "Provides question-answering capabilities for physical AI textbook using "
        "semantic search (Qdrant) and LLM generation (Cohere)."
    ),
    version="1.0.0",
    docs_url="/docs",  # Swagger UI
    redoc_url="/redoc",  # ReDoc alternative
    openapi_url="/openapi.json",  # OpenAPI schema
    lifespan=lifespan,  # Startup/shutdown events
)

# Configure CORS middleware
# ⚠️ USER TODO: Update ALLOWED_ORIGINS in .env to include your frontend URL
setup_cors(app)

# Add request logging middleware
app.middleware("http")(log_requests)

# Include API routers
app.include_router(query_router)
app.include_router(health_router)


@app.get(
    "/",
    tags=["Root"],
    summary="Root endpoint",
    description="Verify that the backend is running"
)
def read_root():
    """
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

    Returns:
        Dictionary with welcome message and links
    """
    return {
        "message": "RAG Chatbot backend is running",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/api/health",
        "query": "POST /api/query",
    }


# ============================================================================
# DEVELOPMENT NOTES
# ============================================================================
#
# Project Structure:
# ------------------
# app/
# ├── __init__.py           - Package initialization
# ├── main.py               - This file (FastAPI app)
# ├── api/                  - API endpoints
# │   ├── __init__.py
# │   ├── query.py          - POST /api/query (RAG endpoint)
# │   └── health.py         - GET /api/health (health check)
# ├── models/               - Pydantic models
# │   ├── __init__.py
# │   ├── query.py          - QueryRequest, QueryResponse, Citation
# │   └── health.py         - HealthResponse
# ├── services/             - Business logic and external integrations
# │   ├── __init__.py
# │   ├── qdrant_service.py - Qdrant vector database client
# │   ├── cohere_service.py - Cohere API client (embeddings + generation)
# │   └── rag_service.py    - RAG orchestration service
# └── middleware/           - Request/response middleware
#     ├── __init__.py
#     ├── cors.py           - CORS configuration
#     └── logging_middleware.py - Request logging
#
# Key Dependencies:
# -----------------
# - FastAPI: Web framework
# - Uvicorn: ASGI server
# - Qdrant: Vector database for semantic search
# - Cohere: LLM API for embeddings and generation
# - python-dotenv: Environment variable management
#
# API Endpoints:
# --------------
# GET  /                     - Root endpoint (welcome message)
# GET  /docs                 - Swagger UI (interactive API docs)
# GET  /redoc                - ReDoc (alternative API docs)
# GET  /api/health           - Health check
# GET  /api/query/health     - Query service health (Qdrant + Cohere)
# POST /api/query            - RAG question answering
#
# Environment Variables:
# ----------------------
# See .env.example for complete list. Critical ones:
# - COHERE_API_KEY          - Required for embeddings and generation
# - QDRANT_URL              - Required for vector search
# - QDRANT_API_KEY          - Required for Qdrant authentication
# - DATABASE_URL            - Optional for logging (PostgreSQL)
# - ALLOWED_ORIGINS         - Required for CORS (frontend URLs)
#
# Next Steps:
# -----------
# 1. ⚠️ Copy .env.example to .env and fill in your credentials
# 2. ⚠️ Create Qdrant collection "textbook_chunks" (see docs)
# 3. ⚠️ Run book ingestion to populate Qdrant with embeddings
# 4. Start the server: uvicorn app.main:app --reload
# 5. Test: curl http://localhost:8000/api/health
# 6. Access docs: http://localhost:8000/docs
#
# For production deployment, see DEPLOYMENT.md
# ============================================================================
