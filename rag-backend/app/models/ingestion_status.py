"""
Ingestion status model for tracking background ingestion tasks.
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class IngestionStatus(BaseModel):
    """Status of a background ingestion task."""

    task_id: str
    status: str  # "pending" | "running" | "completed" | "failed"
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    chunks_processed: int = 0
    total_chunks: Optional[int] = None
    error_message: Optional[str] = None
    book_version: Optional[str] = None


class IngestionStartResponse(BaseModel):
    """Response when ingestion task is started."""

    task_id: str
    status: str
    message: str
    status_url: str
