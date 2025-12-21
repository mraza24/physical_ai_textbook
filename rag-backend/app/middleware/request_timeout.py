"""
Request timeout middleware for FastAPI.

Enforces <3s total request time for query endpoints.
Optionally provides "still thinking" indicator for slow responses.
"""
import os
import asyncio
from fastapi import Request
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware


class RequestTimeoutMiddleware(BaseHTTPMiddleware):
    """Request timeout middleware."""

    def __init__(self, app, timeout_seconds: int = 3):
        """
        Initialize timeout middleware.

        Args:
            app: FastAPI application
            timeout_seconds: Maximum request duration in seconds (default: 3)
        """
        super().__init__(app)
        self.timeout_seconds = timeout_seconds

    async def dispatch(self, request: Request, call_next):
        """
        Handle request with timeout enforcement.

        Args:
            request: FastAPI request
            call_next: Next middleware/handler in chain

        Returns:
            Response or timeout error
        """
        # Only enforce timeout on query endpoint
        # Ingestion is excluded as it can take several minutes for large books
        if request.url.path != "/api/query":
            return await call_next(request)

        try:
            # Set timeout for request processing
            response = await asyncio.wait_for(
                call_next(request),
                timeout=self.timeout_seconds
            )
            return response

        except asyncio.TimeoutError:
            # Request exceeded timeout
            return JSONResponse(
                status_code=504,
                content={
                    "error": f"The request took longer than {self.timeout_seconds} seconds to process. Please try again with a shorter query or less context.",
                    "error_code": "TIMEOUT",
                    "status_code": 504,
                },
            )


def get_timeout_middleware(app):
    """
    Create timeout middleware with configuration from environment.

    Args:
        app: FastAPI application

    Returns:
        RequestTimeoutMiddleware instance
    """
    timeout = int(os.getenv("REQUEST_TIMEOUT_SECONDS", "3"))
    return RequestTimeoutMiddleware(app, timeout_seconds=timeout)
