"""
Health check endpoint for monitoring and readiness checks.
"""
from datetime import datetime
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/api", tags=["health"])

class HealthResponse(BaseModel):
    """Health check response model."""
    status: str
    timestamp: str
    message: str = "Service is healthy"

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.

    Returns:
        HealthResponse: Service health status with timestamp
    """
    return HealthResponse(
        status="healthy",
        timestamp=datetime.utcnow().isoformat() + "Z",
    )

@router.get("/health/ready", response_model=HealthResponse)
async def readiness_check():
    """
    Readiness check endpoint - verifies service dependencies.

    Returns:
        HealthResponse: Service readiness status

    Raises:
        HTTPException: If service dependencies are not ready
    """
    # In the future, this will check database, Qdrant, and Cohere connectivity
    # For now, return healthy if the service is running
    return HealthResponse(
        status="ready",
        timestamp=datetime.utcnow().isoformat() + "Z",
        message="Service is ready to accept requests",
    )
