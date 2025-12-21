"""
Chunk model for text chunks with metadata.
Represents a semantically-aware segment of book content.
"""
from typing import Optional
from pydantic import BaseModel, Field
import uuid


class Chunk(BaseModel):
    """
    Text chunk model with associated metadata.

    Represents a semantically-aware chunk of book content (200-500 tokens target)
    with metadata for retrieval and citation.
    """

    chunk_id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique identifier for the chunk"
    )

    text_content: str = Field(
        ...,
        description="The actual text content of the chunk",
        min_length=1
    )

    chapter: str = Field(
        ...,
        description="Chapter title or identifier",
        max_length=255
    )

    section: Optional[str] = Field(
        default=None,
        description="Section title within the chapter",
        max_length=255
    )

    subsection: Optional[str] = Field(
        default=None,
        description="Subsection title within the section",
        max_length=255
    )

    content_type: str = Field(
        ...,
        description="Type of content: text|glossary|code|reference|figure_caption|exercise",
        pattern="^(text|glossary|code|reference|figure_caption|exercise)$"
    )

    paragraph_id: str = Field(
        ...,
        description="Paragraph identifier in format 'chapter-section-paragraph-N'",
        max_length=255
    )

    deep_link_url: str = Field(
        ...,
        description="Deep link URL to the exact paragraph in Docusaurus",
        max_length=500
    )

    token_count: int = Field(
        ...,
        description="Number of tokens in the chunk",
        ge=0
    )

    overlap_start: int = Field(
        default=0,
        description="Number of overlapping tokens from the previous chunk",
        ge=0
    )

    overlap_end: int = Field(
        default=0,
        description="Number of overlapping tokens with the next chunk",
        ge=0
    )

    book_version: str = Field(
        default="1.0.0",
        description="Version of the book content",
        max_length=50
    )

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "chunk_id": "550e8400-e29b-41d4-a716-446655440000",
                "text_content": "Forward kinematics is the process of computing the position and orientation of the robot's end-effector from its joint parameters.",
                "chapter": "Chapter 2: Robot Kinematics",
                "section": "2.1 Introduction to Kinematics",
                "subsection": "2.1.1 Forward Kinematics",
                "content_type": "text",
                "paragraph_id": "ch2-s1-p3",
                "deep_link_url": "http://localhost:3000/docs/chapter2/kinematics#ch2-s1-p3",
                "token_count": 28,
                "overlap_start": 0,
                "overlap_end": 5,
                "book_version": "1.0.0"
            }
        }
