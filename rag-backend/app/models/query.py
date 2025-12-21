"""
Query model for user questions.
"""
from typing import Optional
from pydantic import BaseModel, Field


class Query(BaseModel):
    """
    User query model.

    Represents a user's question or request for information, optionally with
    selected text from the book as context.
    """

    query_text: str = Field(
        ...,
        description="The user's question or query",
        min_length=10,
        max_length=500
    )

    selected_text: Optional[str] = Field(
        default=None,
        description="Optional text selected from the book for context",
        max_length=5000
    )

    session_id: Optional[str] = Field(
        default=None,
        description="Optional session ID for tracking user sessions"
    )

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "query_text": "Can you explain forward kinematics in simpler terms?",
                "selected_text": "Forward kinematics is the process of computing the position and orientation of the robot's end-effector from its joint parameters.",
                "session_id": "550e8400-e29b-41d4-a716-446655440000"
            }
        }
