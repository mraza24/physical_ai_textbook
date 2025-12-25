"""
Query endpoint for RAG question answering
"""

import logging
from fastapi import APIRouter, HTTPException, status
from app.models.query import QueryRequest, QueryResponse, ErrorResponse

logger = logging.getLogger(__name__)

# Create APIRouter for query endpoints
router = APIRouter(
    prefix="/api",
    tags=["Query"],
)

# Initialize RAG service (singleton pattern)
rag_service = None


def get_rag_service() -> "RAGService":
    """
    Get or create RAG service instance (lazy initialization)

    Returns:
        RAGService instance

    Raises:
        HTTPException: If service initialization fails (missing credentials)
    """
    global rag_service

    if rag_service is None:
        try:
            from app.services.rag_service import RAGService  # lazy import to avoid circular dependency
            rag_service = RAGService()
            logger.info("RAG service initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize RAG service: {e}")
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Service initialization failed: {str(e)}. "
                       f"Check COHERE_API_KEY and QDRANT_URL in .env"
            )

    return rag_service


@router.post(
    "/query",
    response_model=QueryResponse,
    responses={
        400: {"model": ErrorResponse, "description": "Validation error"},
        500: {"model": ErrorResponse, "description": "Internal server error"},
    },
    summary="Query the RAG chatbot",
    description="Submit a question to get an answer from the physical AI textbook using RAG"
)
async def query_textbook(request: QueryRequest) -> QueryResponse:
    """
    Process a user query and return an answer with citations
    """
    try:
        logger.info(f"Received query: {request.query_text[:50]}...")

        # Get RAG service (lazy init)
        service = get_rag_service()

        # Process query
        result = await service.query(
            query_text=request.query_text,
            selected_text=request.selected_text
        )

        return QueryResponse(**result)

    except ValueError as e:
        logger.warning(f"Validation error: {e}")
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )

    except Exception as e:
        logger.error(f"Query processing failed: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )


@router.get(
    "/query/health",
    summary="Check query service health",
    description="Verify that Qdrant and Cohere services are accessible"
)
async def query_health():
    """
    Health check for query dependencies
    """
    try:
        service = get_rag_service()
        health_status = service.health_check()
        return health_status

    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {
            "cohere": False,
            "qdrant": False,
            "overall": False,
            "error": str(e)
        }
