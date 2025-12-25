"""
Health check endpoint for monitoring and deployment verification
"""

import logging
from datetime import datetime
from fastapi import APIRouter

from app.models.health import HealthResponse

logger = logging.getLogger(__name__)

# Create APIRouter for health endpoints
router = APIRouter(
    prefix="/api",
    tags=["Health"],
)


@router.get(
    "/health",
    response_model=HealthResponse,
    summary="Health check",
    description="Check if the backend service is running and healthy"
)
async def health_check() -> HealthResponse:
    """
    Health check endpoint for deployment verification and monitoring

    **Response:**
    ```json
    {
        "status": "healthy",
        "timestamp": "2025-12-22T10:30:00Z",
        "message": "Service is healthy"
    }
    ```

    **Usage:**
    - **Deployment verification**: Call after deployment to ensure service is up
    - **Uptime monitoring**: Use with services like UptimeRobot to prevent spin-down
    - **Load balancer health checks**: Configure ALB/NLB to hit this endpoint

    **Example with curl:**
    ```bash
    curl https://your-backend.com/api/health
    ```

    Returns:
        HealthResponse with status and timestamp
    """
    logger.debug("Health check requested")

    return HealthResponse(
        status="healthy",
        timestamp=datetime.utcnow(),
        message="Service is healthy"
    )
