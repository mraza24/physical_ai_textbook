"""
RetrievedChunk model for chunks retrieved from vector search.
"""
from pydantic import BaseModel, Field


class RetrievedChunk(BaseModel):
    """
    Retrieved chunk model with similarity score.

    Represents a chunk retrieved from vector search with its relevance score.
    """

    chunk_id: str = Field(..., description="Unique chunk identifier")
    text_content: str = Field(..., description="Chunk text content")
    chapter: str = Field(..., description="Chapter title")
    section: str = Field(default="", description="Section title")
    content_type: str = Field(..., description="Content type")
    paragraph_id: str = Field(..., description="Paragraph identifier")
    deep_link_url: str = Field(..., description="Deep link URL to paragraph")
    similarity_score: float = Field(..., description="Similarity score (0.0-1.0)", ge=0.0, le=1.0)

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
                "text_content": "Forward kinematics is the process...",
                "chapter": "Chapter 2: Robot Kinematics",
                "section": "2.1 Introduction to Kinematics",
                "content_type": "text",
                "paragraph_id": "ch2-s1-p3",
                "deep_link_url": "http://localhost:3000/docs/chapter2/kinematics#ch2-s1-p3",
                "similarity_score": 0.92,
            }
        }
