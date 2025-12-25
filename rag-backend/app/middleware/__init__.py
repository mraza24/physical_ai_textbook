"""
Middleware for request processing, CORS, logging, etc.
"""

from app.middleware.cors import setup_cors
from app.middleware.logging_middleware import setup_logging

__all__ = [
    "setup_cors",
    "setup_logging",
]
