"""
Ingestion API endpoint for loading and indexing book content.

Handles:
- Admin-only access (requires ADMIN_API_KEY)
- Full ingestion pipeline orchestration
- Logging to database
- Error handling and progress reporting
"""
import os
import time
import uuid
from datetime import datetime
from typing import Optional
from fastapi import APIRouter, Header, HTTPException, status
from pydantic import BaseModel, Field

from app.services.content_loader import load_book_files
from app.services.chunker import semantic_chunk
from app.services.embedding_service import generate_embeddings
from app.services.vector_store import upsert_chunks
from app.database import execute_write


router = APIRouter(prefix="/api", tags=["ingestion"])


class IngestRequest(BaseModel):
    """Ingestion request model."""

    book_path: str = Field(
        ...,
        description="Path to the Docusaurus docs/ directory",
        example="/path/to/textbook/docs"
    )

    chunk_size: Optional[int] = Field(
        default=None,
        description="Target chunk size in tokens (overrides env default)",
        ge=100,
        le=1000
    )

    chunk_overlap: Optional[float] = Field(
        default=None,
        description="Chunk overlap percentage (overrides env default)",
        ge=0.0,
        le=0.5
    )


class IngestResponse(BaseModel):
    """Ingestion response model."""

    status: str = Field(..., description="Ingestion status: success|error")
    ingestion_id: str = Field(..., description="Unique ingestion ID")
    chunks_processed: int = Field(..., description="Number of chunks processed")
    duration_seconds: float = Field(..., description="Total duration in seconds")
    errors: Optional[str] = Field(default=None, description="Error messages if any")
    message: str = Field(..., description="Human-readable status message")


def verify_admin_key(x_admin_api_key: str = Header(None)) -> bool:
    """
    Verify admin API key from request header.

    Args:
        x_admin_api_key: Admin API key from header

    Raises:
        HTTPException: If key is missing or invalid
    """
    admin_key = os.getenv("ADMIN_API_KEY")

    if not admin_key:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="ADMIN_API_KEY not configured on server",
        )

    if not x_admin_api_key:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing X-Admin-API-Key header",
        )

    if x_admin_api_key != admin_key:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Invalid admin API key",
        )

    return True


@router.post("/ingest", response_model=IngestResponse)
async def ingest_book_content(
    request: IngestRequest,
    x_admin_api_key: str = Header(None, alias="X-Admin-API-Key"),
):
    """
    Ingest book content and create embeddings.

    This endpoint orchestrates the full ingestion pipeline:
    1. Load book files from Docusaurus docs/ directory
    2. Chunk content at semantic boundaries
    3. Generate embeddings using Cohere API
    4. Upsert chunks and embeddings to Qdrant
    5. Log ingestion summary to database

    **Authentication**: Requires X-Admin-API-Key header with valid admin key.

    Args:
        request: Ingestion request with book path and optional parameters
        x_admin_api_key: Admin API key from header

    Returns:
        IngestResponse with ingestion status and statistics

    Raises:
        HTTPException: If authentication fails or ingestion errors occur
    """
    # Verify admin access
    verify_admin_key(x_admin_api_key)

    # Generate ingestion ID
    ingestion_id = str(uuid.uuid4())
    start_time = time.time()
    errors = []

    try:
        # Get chunking parameters from request or environment
        chunk_size = request.chunk_size or int(os.getenv("CHUNK_SIZE", "400"))
        chunk_overlap = request.chunk_overlap or float(os.getenv("CHUNK_OVERLAP", "0.15"))
        min_chunk_size = int(os.getenv("MIN_CHUNK_SIZE", "100"))
        max_chunk_size = int(os.getenv("MAX_CHUNK_SIZE", "700"))

        print(f"[{ingestion_id}] Starting ingestion from: {request.book_path}")
        print(f"[{ingestion_id}] Chunk config: size={chunk_size}, overlap={chunk_overlap}")

        # Step 1: Load book files
        print(f"[{ingestion_id}] Loading book files...")
        book_contents = load_book_files(request.book_path)
        print(f"[{ingestion_id}] Loaded {len(book_contents)} book content items")

        if not book_contents:
            raise ValueError(f"No content found in book path: {request.book_path}")

        # Step 2: Chunk content
        print(f"[{ingestion_id}] Chunking content...")
        all_chunks = []
        for content in book_contents:
            try:
                chunks = semantic_chunk(
                    content,
                    target_size=chunk_size,
                    min_size=min_chunk_size,
                    max_size=max_chunk_size,
                    overlap=chunk_overlap,
                )
                all_chunks.extend(chunks)
            except Exception as e:
                error_msg = f"Error chunking {content.file_path}: {e}"
                print(f"[{ingestion_id}] {error_msg}")
                errors.append(error_msg)

        print(f"[{ingestion_id}] Created {len(all_chunks)} chunks")

        if not all_chunks:
            raise ValueError("No chunks created from book content")

        # Step 3: Generate embeddings
        print(f"[{ingestion_id}] Generating embeddings...")
        chunk_embedding_pairs = generate_embeddings(all_chunks)
        print(f"[{ingestion_id}] Generated {len(chunk_embedding_pairs)} embeddings")

        # Extract chunks and embeddings
        chunks = [pair[0] for pair in chunk_embedding_pairs]
        embeddings = [pair[1] for pair in chunk_embedding_pairs]

        # Step 4: Upsert to Qdrant
        print(f"[{ingestion_id}] Upserting to Qdrant...")
        upserted_count = upsert_chunks(chunks, embeddings)
        print(f"[{ingestion_id}] Upserted {upserted_count} chunks to Qdrant")

        # Calculate duration
        duration = time.time() - start_time

        # Step 5: Log to database
        try:
            await log_ingestion(
                ingestion_id=ingestion_id,
                chunks_processed=upserted_count,
                errors="\n".join(errors) if errors else None,
                status="success" if not errors else "partial_success",
            )
        except Exception as e:
            print(f"[{ingestion_id}] Warning: Failed to log to database: {e}")

        # Return success response
        return IngestResponse(
            status="success" if not errors else "partial_success",
            ingestion_id=ingestion_id,
            chunks_processed=upserted_count,
            duration_seconds=round(duration, 2),
            errors="\n".join(errors) if errors else None,
            message=f"Successfully ingested {upserted_count} chunks in {duration:.2f}s",
        )

    except Exception as e:
        # Calculate duration
        duration = time.time() - start_time

        error_message = str(e)
        print(f"[{ingestion_id}] Ingestion failed: {error_message}")

        # Log failure to database
        try:
            await log_ingestion(
                ingestion_id=ingestion_id,
                chunks_processed=0,
                errors=error_message,
                status="error",
            )
        except Exception as log_error:
            print(f"[{ingestion_id}] Warning: Failed to log error to database: {log_error}")

        # Return error response
        return IngestResponse(
            status="error",
            ingestion_id=ingestion_id,
            chunks_processed=0,
            duration_seconds=round(duration, 2),
            errors=error_message,
            message=f"Ingestion failed: {error_message}",
        )


async def log_ingestion(
    ingestion_id: str,
    chunks_processed: int,
    errors: Optional[str],
    status: str,
):
    """
    Log ingestion operation to database.

    Args:
        ingestion_id: Unique ingestion ID
        chunks_processed: Number of chunks processed
        errors: Error messages if any
        status: Ingestion status (success|partial_success|error)
    """
    query = """
        INSERT INTO ingestion_logs (
            ingestion_id,
            timestamp,
            chunks_processed,
            errors,
            status
        ) VALUES ($1, $2, $3, $4, $5)
    """

    await execute_write(
        query,
        ingestion_id,
        datetime.utcnow(),
        chunks_processed,
        errors,
        status,
    )

# Background Ingestion Endpoints

from fastapi import BackgroundTasks
from app.models.ingestion_status import IngestionStatus, IngestionStartResponse
from app.services.background_ingestion import (
    create_ingestion_task,
    get_ingestion_status,
    run_ingestion_task,
)


@router.post("/ingest/background", response_model=IngestionStartResponse)
async def ingest_book_background(
    request: IngestRequest,
    background_tasks: BackgroundTasks,
    x_admin_api_key: str = Header(None, alias="X-Admin-API-Key"),
):
    """
    Start book ingestion in background.

    This endpoint immediately returns with a task ID, allowing the client
    to poll for status. Useful for large books that take >30s to ingest.

    Requires admin authentication via X-Admin-API-Key header.

    Args:
        request: Ingestion request with book_path and optional parameters
        background_tasks: FastAPI background tasks
        x_admin_api_key: Admin API key

    Returns:
        IngestionStartResponse with task_id and status_url

    Raises:
        HTTPException: If admin key is invalid or missing
    """
    # Verify admin key
    verify_admin_key(x_admin_api_key)

    # Create task
    task_id = create_ingestion_task(
        book_path=request.book_path,
        book_version="1.0.0",  # TODO: Add to request model
    )

    # Start ingestion in background
    background_tasks.add_task(
        run_ingestion_task,
        task_id,
        request.chunk_size,
        request.chunk_overlap,
    )

    return IngestionStartResponse(
        task_id=task_id,
        status="started",
        message="Ingestion started in background. Check status at /api/ingest/status/{task_id}",
        status_url=f"/api/ingest/status/{task_id}",
    )


@router.get("/ingest/status/{task_id}", response_model=IngestionStatus)
async def get_ingestion_task_status(task_id: str):
    """
    Get status of a background ingestion task.

    Args:
        task_id: Task ID from /ingest/background response

    Returns:
        IngestionStatus with current task status

    Raises:
        HTTPException: If task_id not found
    """
    status = get_ingestion_status(task_id)

    if not status:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Ingestion task {task_id} not found",
        )

    return IngestionStatus(**status)
