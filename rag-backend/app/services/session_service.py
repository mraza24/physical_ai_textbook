"""
Session service for managing user sessions.

Handles:
- Session ID generation
- Session tracking in database
- Session metadata updates
"""
import uuid
from datetime import datetime
from typing import Optional

from app.database import execute_one, execute_write


async def create_session() -> str:
    """
    Create a new session and store in database.

    Returns:
        Session ID (UUID)
    """
    session_id = str(uuid.uuid4())

    query = """
        INSERT INTO sessions (
            session_id,
            created_at,
            last_active_at,
            query_count
        ) VALUES ($1, $2, $3, $4)
    """

    await execute_write(
        query,
        session_id,
        datetime.utcnow(),
        datetime.utcnow(),
        0,
    )

    return session_id


async def update_session(session_id: str) -> None:
    """
    Update session with last active time and increment query count.

    Args:
        session_id: Session ID to update
    """
    query = """
        UPDATE sessions
        SET last_active_at = $1,
            query_count = query_count + 1
        WHERE session_id = $2
    """

    await execute_write(
        query,
        datetime.utcnow(),
        session_id,
    )


async def get_session(session_id: str) -> Optional[dict]:
    """
    Get session information from database.

    Args:
        session_id: Session ID to retrieve

    Returns:
        Session dictionary or None if not found
    """
    query = """
        SELECT session_id, created_at, last_active_at, query_count
        FROM sessions
        WHERE session_id = $1
    """

    row = await execute_one(query, session_id)

    if row:
        return {
            "session_id": str(row["session_id"]),
            "created_at": row["created_at"].isoformat(),
            "last_active_at": row["last_active_at"].isoformat(),
            "query_count": row["query_count"],
        }

    return None
