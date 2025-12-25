"""
API routes for RAG Chatbot backend
"""

from app.api.query import router as query_router
from app.api.health import router as health_router

__all__ = [
    "query_router",
    "health_router",
]
