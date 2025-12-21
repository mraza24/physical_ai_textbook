"""
BookContent model for raw book content before chunking.
Represents a single piece of book content loaded from the Docusaurus docs.
"""
from typing import Optional
from pydantic import BaseModel, Field


class BookContent(BaseModel):
    """
    Raw book content model.

    Represents content loaded from a single markdown/MDX file before chunking,
    with extracted metadata from the file structure and frontmatter.
    """

    file_path: str = Field(
        ...,
        description="Relative file path from docs/ directory",
        max_length=500
    )

    content: str = Field(
        ...,
        description="Raw content text (markdown/MDX)",
        min_length=1
    )

    content_type: str = Field(
        ...,
        description="Type of content: text|glossary|code|reference|figure_caption|exercise",
        pattern="^(text|glossary|code|reference|figure_caption|exercise)$"
    )

    chapter: str = Field(
        ...,
        description="Chapter title or identifier extracted from directory structure",
        max_length=255
    )

    section: Optional[str] = Field(
        default=None,
        description="Section title extracted from headings or file structure",
        max_length=255
    )

    paragraph_id: Optional[str] = Field(
        default=None,
        description="Paragraph identifier if applicable",
        max_length=255
    )

    class Config:
        """Pydantic model configuration."""
        json_schema_extra = {
            "example": {
                "file_path": "module1/intro.md",
                "content": "# Introduction to Robotics\n\nRobotics is the field of...",
                "content_type": "text",
                "chapter": "Module 1: Introduction",
                "section": "Introduction to Robotics",
                "paragraph_id": None
            }
        }
