"""
Query request and response models for RAG API
"""

from typing import List, Optional
from pydantic import BaseModel, Field, field_validator


class QueryRequest(BaseModel):
    """
    Request model for RAG query endpoint

    Example:
        {
            "query_text": "What is forward kinematics?",
            "selected_text": null
        }
    """
    query_text: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="User question in English (1-500 characters)"
    )
    selected_text: Optional[str] = Field(
        None,
        description="Optional highlighted text from textbook for context"
    )

    @field_validator("query_text")
    @classmethod
    def validate_non_empty(cls, v: str) -> str:
        """Ensure query is not just whitespace"""
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()


class Citation(BaseModel):
    """
    Citation model for source references

    Example:
        {
            "section_title": "Module 1: Robot Kinematics",
            "deep_link_url": "/docs/module1/kinematics#forward-kinematics",
            "chunk_count": 3
        }
    """
    section_title: str = Field(
        ...,
        description="Title of the textbook section"
    )
    deep_link_url: str = Field(
        ...,
        description="URL path to textbook section (relative or absolute)"
    )
    chunk_count: int = Field(
        ...,
        ge=1,
        description="Number of chunks retrieved from this section"
    )


class QueryResponse(BaseModel):
    """
    Response model for RAG query endpoint

    Example:
        {
            "answer": "Forward kinematics is the process of...",
            "citations": [
                {
                    "section_title": "Module 1: Robot Kinematics",
                    "deep_link_url": "/docs/module1/kinematics#forward-kinematics",
                    "chunk_count": 3
                }
            ],
            "confidence": 0.87,
            "retrieved_chunk_count": 5,
            "processing_time_ms": 342
        }
    """
    answer: str = Field(
        ...,
        description="Generated answer to the query"
    )
    citations: List[Citation] = Field(
        default_factory=list,
        description="List of source citations with deep links"
    )
    confidence: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence score of the answer (0.0-1.0)"
    )
    retrieved_chunk_count: int = Field(
        ...,
        ge=0,
        description="Total number of chunks retrieved"
    )
    processing_time_ms: int = Field(
        ...,
        ge=0,
        description="Total processing time in milliseconds"
    )


class ErrorResponse(BaseModel):
    """
    Error response model

    Example:
        {
            "error": "Query validation failed",
            "error_code": "VALIDATION_ERROR",
            "status_code": 400
        }
    """
    error: str = Field(
        ...,
        description="Human-readable error message"
    )
    error_code: str = Field(
        ...,
        description="Machine-readable error code"
    )
    status_code: int = Field(
        ...,
        ge=400,
        le=599,
        description="HTTP status code"
    )
