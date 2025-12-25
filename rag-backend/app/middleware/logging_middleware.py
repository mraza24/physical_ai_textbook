"""
Logging Middleware
Configures application logging and request/response logging
"""

import logging
import sys
from fastapi import FastAPI, Request
import time

logger = logging.getLogger(__name__)


def setup_logging():
    """
    Configure application-wide logging

    Sets up console logging with INFO level and formatted output.
    Logs are written to stdout for easy access in cloud deployments.

    ⚠️ USER TODO: For production, consider adding file logging or external
    logging service (e.g., Datadog, LogDNA, CloudWatch)
    """
    # Configure root logger
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout)  # Log to stdout for cloud platforms
        ]
    )

    # Set log levels for noisy libraries
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.INFO)
    logging.getLogger("cohere").setLevel(logging.INFO)

    logger.info("Logging configured successfully")


async def log_requests(request: Request, call_next):
    """
    Middleware to log incoming requests and responses

    Args:
        request: Incoming HTTP request
        call_next: Next middleware/route handler

    Returns:
        HTTP response

    Logs:
        - Request method, path, and client IP
        - Response status code and processing time
    """
    start_time = time.time()

    # Log incoming request
    logger.info(
        f"Request: {request.method} {request.url.path} "
        f"from {request.client.host if request.client else 'unknown'}"
    )

    # Process request
    response = await call_next(request)

    # Calculate processing time
    process_time = (time.time() - start_time) * 1000  # Convert to ms

    # Log response
    logger.info(
        f"Response: {response.status_code} "
        f"(processed in {process_time:.2f}ms)"
    )

    # Add processing time to response headers (useful for debugging)
    response.headers["X-Process-Time"] = str(round(process_time, 2))

    return response
