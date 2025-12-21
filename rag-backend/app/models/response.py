"""
ChatResponse model for chatbot answers.
"""
from typing import List, Optional
from pydantic import BaseModel, Field

from app.models.citation import Citation


class ChatResponse(BaseModel):
    """
    Chatbot response model.

    Represents the chatbot's answer to a user query with source citations
    and confidence indicators.
    """

    answer: str = Field(
        ...,
        description="The chatbot's answer to the query",
        max_length=2000
    )

    citations: List[Citation] = Field(
        default_factory=list,
        description="List of source citations used in the answer"
    )

    confidence: str = Field(
        ...,
        description="Confidence level: high|medium|low",
        pattern="^(high|medium|low)$"
    )

    session_id: Optional[str] = Field(
        default=None,
        description="Session ID for tracking"
    )

    error: Optional[str] = Field(
        default=None,
        description="Error message if query failed"
    )

    error_code: Optional[str] = Field(
        default=None,
        description="Error code for programmatic handling"
    )

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "answer": "Forward kinematics is the mathematical process of determining where a robot's end-effector (like a gripper or tool) is located in space, given the angles of all its joints. Think of it like this: if you know how much each joint of your arm is bent, you can figure out where your hand is positioned.",
                "citations": [
                    {
                        "section_title": "Chapter 2: Robot Kinematics",
                        "deep_link_url": "http://localhost:3000/docs/chapter2/kinematics#ch2-s1-p3",
                        "chunk_count": 2
                    }
                ],
                "confidence": "high",
                "session_id": "550e8400-e29b-41d4-a716-446655440000",
                "error": None,
                "error_code": None
            }
        }
