"""
CORS (Cross-Origin Resource Sharing) Middleware
Allows frontend to make requests to backend API
"""

import os
import logging
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

logger = logging.getLogger(__name__)


def setup_cors(app: FastAPI):
    """
    Configure CORS middleware for FastAPI app

    Args:
        app: FastAPI application instance

    Environment Variables:
        - ALLOWED_ORIGINS: Comma-separated list of allowed origins
          Example: http://localhost:3000,https://my-app.vercel.app

    ⚠️ USER TODO: Update ALLOWED_ORIGINS in .env to include your frontend URL
    For development: http://localhost:3000
    For production: https://your-frontend.vercel.app
    """
    # Get allowed origins from environment
    allowed_origins_str = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000")
    allowed_origins = [origin.strip() for origin in allowed_origins_str.split(",")]

    logger.info(f"Configuring CORS with allowed origins: {allowed_origins}")

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=allowed_origins,  # List of allowed origins
        allow_credentials=True,  # Allow cookies/auth headers
        allow_methods=["*"],  # Allow all HTTP methods (GET, POST, etc.)
        allow_headers=["*"],  # Allow all headers
    )

    logger.info("CORS middleware configured successfully")
