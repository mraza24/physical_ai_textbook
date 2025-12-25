"""
Health check response model
"""

from datetime import datetime
from pydantic import BaseModel, Field


class HealthResponse(BaseModel):
    """
    Health check response

    Example:
        {
            "status": "healthy",
            "timestamp": "2025-12-22T10:30:00Z",
            "message": "Service is healthy"
        }
    """
    status: str = Field(
        default="healthy",
        description="Service status: healthy, degraded, or unhealthy"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Current server timestamp"
    )
    message: str = Field(
        default="Service is healthy",
        description="Status message"
    )
