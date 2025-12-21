"""
Global error handler middleware for FastAPI.

Handles:
- HTTPException (validation errors, 404s, etc.)
- Cohere API errors (timeouts, rate limits, service errors)
- Qdrant connection errors
- Database connection errors
- Unexpected exceptions

Returns user-friendly error messages without exposing internal details.
"""
import sys
import traceback
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
import cohere
from qdrant_client.http.exceptions import UnexpectedResponse as QdrantError
import asyncpg


class ErrorHandlerMiddleware(BaseHTTPMiddleware):
    """Global error handler middleware."""

    async def dispatch(self, request: Request, call_next):
        """
        Handle request and catch all exceptions.

        Args:
            request: FastAPI request
            call_next: Next middleware/handler in chain

        Returns:
            Response with error handling
        """
        try:
            response = await call_next(request)
            return response

        except HTTPException as exc:
            # HTTP exceptions (validation errors, 404s, etc.)
            return JSONResponse(
                status_code=exc.status_code,
                content={
                    "error": exc.detail,
                    "error_code": "HTTP_ERROR",
                    "status_code": exc.status_code,
                },
            )

        except cohere.errors.TooManyRequestsError as exc:
            # Cohere rate limit exceeded
            print(f"Cohere rate limit exceeded: {exc}", file=sys.stderr)
            return JSONResponse(
                status_code=503,
                content={
                    "error": "The chatbot is temporarily unavailable due to high demand. Please try again in a moment.",
                    "error_code": "RATE_LIMIT_EXCEEDED",
                    "status_code": 503,
                },
            )

        except (
            cohere.errors.InternalServerError,
            cohere.errors.ServiceUnavailableError,
        ) as exc:
            # Cohere API errors
            print(f"Cohere API error: {exc}", file=sys.stderr)
            return JSONResponse(
                status_code=503,
                content={
                    "error": "The chatbot is temporarily unavailable. Please try again shortly.",
                    "error_code": "SERVICE_UNAVAILABLE",
                    "status_code": 503,
                },
            )

        except cohere.errors.CohereError as exc:
            # Other Cohere errors
            print(f"Cohere error: {exc}", file=sys.stderr)
            return JSONResponse(
                status_code=503,
                content={
                    "error": "The chatbot is temporarily unavailable. Please try again shortly.",
                    "error_code": "AI_SERVICE_ERROR",
                    "status_code": 503,
                },
            )

        except QdrantError as exc:
            # Qdrant connection/query errors
            print(f"Qdrant error: {exc}", file=sys.stderr)
            return JSONResponse(
                status_code=503,
                content={
                    "error": "The chatbot is temporarily unavailable. Please try again shortly.",
                    "error_code": "VECTOR_DB_ERROR",
                    "status_code": 503,
                },
            )

        except (asyncpg.PostgresError, asyncpg.InterfaceError) as exc:
            # Database connection/query errors
            print(f"Database error: {exc}", file=sys.stderr)
            return JSONResponse(
                status_code=503,
                content={
                    "error": "The service is temporarily unavailable. Please try again shortly.",
                    "error_code": "DATABASE_ERROR",
                    "status_code": 503,
                },
            )

        except TimeoutError as exc:
            # Request timeout
            print(f"Request timeout: {exc}", file=sys.stderr)
            return JSONResponse(
                status_code=504,
                content={
                    "error": "The request took too long to process. Please try again with a shorter query.",
                    "error_code": "TIMEOUT",
                    "status_code": 504,
                },
            )

        except Exception as exc:
            # Unexpected errors - log but don't expose details
            print(f"Unexpected error: {exc}", file=sys.stderr)
            traceback.print_exc(file=sys.stderr)

            return JSONResponse(
                status_code=500,
                content={
                    "error": "An unexpected error occurred. Please try again or contact support if the issue persists.",
                    "error_code": "INTERNAL_ERROR",
                    "status_code": 500,
                },
            )
