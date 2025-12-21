"""
RAG Chatbot FastAPI Application
Main entry point for the backend API server.
"""
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Import routers
from app.api import health, ingest, query

# Import middleware
from app.middleware.error_handler import ErrorHandlerMiddleware
from app.middleware.request_timeout import get_timeout_middleware

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Backend API for AI-Native Textbook RAG Chatbot",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
)

# Get allowed origins from environment
allowed_origins_str = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000")
allowed_origins = [origin.strip() for origin in allowed_origins_str.split(",")]

# Configure middleware (order matters: last added = first executed)
# 1. CORS (outermost)
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,  
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# 2. Request timeout (before error handler to catch timeouts)
app.add_middleware(get_timeout_middleware(app).__class__, timeout_seconds=int(os.getenv("REQUEST_TIMEOUT_SECONDS", "3")))

# 3. Global error handler (innermost, catches all errors)
app.add_middleware(ErrorHandlerMiddleware)

# Include routers
app.include_router(health.router)
app.include_router(ingest.router)
app.include_router(query.router)

@app.on_event("startup")
async def startup_event():
    """Initialize resources on application startup."""
    print("🚀 RAG Chatbot API starting up...")
    print(f"   Allowed origins: {allowed_origins}")

@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup resources on application shutdown."""
    print("👋 RAG Chatbot API shutting down...")

@app.get("/")
async def root():
    """Root endpoint - API information."""
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "docs": "/api/docs",
    }
