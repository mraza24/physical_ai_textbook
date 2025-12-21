"""
Citation model for source attributions.
"""
from pydantic import BaseModel, Field


class Citation(BaseModel):
    """
    Citation model for source references.

    Represents a source reference to book sections that contributed to an answer.
    Multiple chunks from the same section are grouped into one citation.
    """

    section_title: str = Field(
        ...,
        description="Section title (e.g., 'Chapter 2: Kinematics')",
        max_length=255
    )

    deep_link_url: str = Field(
        ...,
        description="Clickable deep link to the section in Docusaurus",
        max_length=500
    )

    chunk_count: int = Field(
        ...,
        description="Number of chunks from this section used in the answer",
        ge=1
    )

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "section_title": "Chapter 2: Robot Kinematics",
                "deep_link_url": "http://localhost:3000/docs/chapter2/kinematics#ch2-s1-p3",
                "chunk_count": 2
            }
        }
