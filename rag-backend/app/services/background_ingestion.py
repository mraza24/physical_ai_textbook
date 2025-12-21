"""
Background ingestion service for long-running book indexing tasks.

Allows ingestion to run asynchronously without blocking HTTP requests.
"""
import uuid
import time
from datetime import datetime
from typing import Dict, Optional

from app.services.content_loader import load_book_files
from app.services.chunker import semantic_chunk
from app.services.embedding_service import generate_embeddings
from app.services.vector_store import upsert_chunks
from app.database import execute_write


# In-memory task storage (for demo/hackathon - use Redis/database for production)
_ingestion_tasks: Dict[str, dict] = {}


def create_ingestion_task(book_path: str, book_version: str = "1.0.0") -> str:
    """
    Create a new ingestion task.

    Args:
        book_path: Path to book content
        book_version: Version of the book

    Returns:
        Task ID (UUID)
    """
    task_id = str(uuid.uuid4())

    _ingestion_tasks[task_id] = {
        "task_id": task_id,
        "status": "pending",
        "book_path": book_path,
        "book_version": book_version,
        "started_at": None,
        "completed_at": None,
        "chunks_processed": 0,
        "total_chunks": None,
        "error_message": None,
    }

    return task_id


def get_ingestion_status(task_id: str) -> Optional[dict]:
    """
    Get status of an ingestion task.

    Args:
        task_id: Task ID to query

    Returns:
        Task status dict or None if not found
    """
    return _ingestion_tasks.get(task_id)


async def run_ingestion_task(task_id: str, chunk_size: Optional[int] = None, chunk_overlap: Optional[float] = None):
    """
    Run ingestion task in background.

    Args:
        task_id: Task ID to execute
        chunk_size: Optional custom chunk size
        chunk_overlap: Optional custom overlap percentage
    """
    task = _ingestion_tasks.get(task_id)

    if not task:
        print(f"Task {task_id} not found")
        return

    try:
        # Update status to running
        task["status"] = "running"
        task["started_at"] = datetime.utcnow().isoformat()

        print(f"[Ingestion {task_id}] Starting ingestion...")

        # 1. Load book files
        print(f"[Ingestion {task_id}] Loading book files from: {task['book_path']}")
        book_files = load_book_files(task["book_path"])
        print(f"[Ingestion {task_id}] Loaded {len(book_files)} files")

        # 2. Chunk content
        print(f"[Ingestion {task_id}] Chunking content...")
        all_chunks = []
        for book_file in book_files:
            chunks = semantic_chunk(
                book_file,
                target_size=chunk_size or 400,
                chunk_overlap=chunk_overlap or 0.15,
            )
            all_chunks.extend(chunks)

        task["total_chunks"] = len(all_chunks)
        print(f"[Ingestion {task_id}] Created {len(all_chunks)} chunks")

        # 3. Generate embeddings
        print(f"[Ingestion {task_id}] Generating embeddings...")
        chunk_embedding_pairs = generate_embeddings(all_chunks)

        chunks = [pair[0] for pair in chunk_embedding_pairs]
        embeddings = [pair[1] for pair in chunk_embedding_pairs]

        # 4. Upsert to vector store
        print(f"[Ingestion {task_id}] Upserting to vector store...")
        chunks_upserted = upsert_chunks(chunks, embeddings)

        task["chunks_processed"] = chunks_upserted

        # 5. Log to database
        ingestion_id = str(uuid.uuid4())
        await execute_write(
            """
            INSERT INTO ingestion_logs (ingestion_id, timestamp, chunks_processed, errors, status)
            VALUES ($1, $2, $3, $4, $5)
            """,
            ingestion_id,
            datetime.utcnow(),
            chunks_upserted,
            None,
            "completed",
        )

        # Mark task as completed
        task["status"] = "completed"
        task["completed_at"] = datetime.utcnow().isoformat()

        print(f"[Ingestion {task_id}] Completed successfully: {chunks_upserted} chunks")

    except Exception as e:
        # Mark task as failed
        task["status"] = "failed"
        task["error_message"] = str(e)
        task["completed_at"] = datetime.utcnow().isoformat()

        print(f"[Ingestion {task_id}] Failed with error: {e}")

        # Log error to database
        try:
            ingestion_id = str(uuid.uuid4())
            await execute_write(
                """
                INSERT INTO ingestion_logs (ingestion_id, timestamp, chunks_processed, errors, status)
                VALUES ($1, $2, $3, $4, $5)
                """,
                ingestion_id,
                datetime.utcnow(),
                task.get("chunks_processed", 0),
                str(e),
                "failed",
            )
        except Exception as log_error:
            print(f"[Ingestion {task_id}] Failed to log error: {log_error}")
