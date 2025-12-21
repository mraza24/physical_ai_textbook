"""
Query API endpoint for answering user questions.

Handles:
- Query validation and processing
- Full query pipeline orchestration
- Database logging
- Error handling
"""
import uuid
from datetime import datetime
from fastapi import APIRouter, HTTPException, status
import os

from app.models.query import Query
from app.models.response import ChatResponse
from app.services.query_validator import validate_query
from app.services.retriever import retrieve_chunks
from app.services.context_assembler import assemble_context
from app.services.answer_generator import generate_answer
from app.services.citation_formatter import format_citations
from app.services.session_service import create_session, update_session, get_session
from app.database import execute_write


router = APIRouter(prefix="/api", tags=["query"])


@router.post("/query", response_model=ChatResponse)
async def process_query(query: Query):
    """
    Process a user query and return an answer with citations.

    Orchestrates the full query pipeline:
    1. Validate query (sanitization, injection detection, language, length)
    2. Retrieve relevant chunks using hybrid search
    3. Assemble context from chunks and selected text
    4. Generate answer using Cohere Chat API
    5. Format citations
    6. Log query and response to database
    7. Return ChatResponse

    Args:
        query: Query object with query_text, optional selected_text, optional session_id

    Returns:
        ChatResponse with answer, citations, and confidence

    Raises:
        HTTPException: If query validation fails or errors occur
    """
    query_id = str(uuid.uuid4())

    try:
        # Step 1: Validate query
        is_valid, error_message = validate_query(query.query_text, query.selected_text)

        if not is_valid:
            return ChatResponse(
                answer="",
                citations=[],
                confidence="low",
                session_id=query.session_id,
                error=error_message,
                error_code="VALIDATION_ERROR",
            )

        # Handle session
        session_id = query.session_id
        if not session_id:
            session_id = await create_session()
        else:
            # Check if session exists
            existing_session = await get_session(session_id)
            if not existing_session:
                session_id = await create_session()

        # Update session activity
        await update_session(session_id)

        # Step 2: Retrieve relevant chunks
        try:
            retrieved_chunks = retrieve_chunks(
                query_text=query.query_text,
                selected_text=query.selected_text,
            )
        except Exception as e:
            print(f"Error retrieving chunks: {e}")
            return ChatResponse(
                answer="",
                citations=[],
                confidence="low",
                session_id=session_id,
                error="The chatbot is temporarily unavailable. Please try again shortly.",
                error_code="RETRIEVAL_ERROR",
            )

        # Step 3: Check if sufficient context was retrieved
        similarity_threshold = float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))

        if not retrieved_chunks:
            answer = "I cannot find sufficient information in the textbook to answer this question accurately."

            # Log query and response
            await log_query_and_response(
                query_id=query_id,
                session_id=session_id,
                query_text=query.query_text,
                selected_text=query.selected_text,
                answer=answer,
                token_count=len(answer.split()),
                confidence_score=0.0,
            )

            return ChatResponse(
                answer=answer,
                citations=[],
                confidence="low",
                session_id=session_id,
            )

        # Step 4: Assemble context
        context, chunks_used = assemble_context(
            query_text=query.query_text,
            retrieved_chunks=retrieved_chunks,
            selected_text=query.selected_text,
        )

        # Step 5: Generate answer
        try:
            answer, token_count = generate_answer(context, query.query_text)
        except Exception as e:
            print(f"Error generating answer: {e}")
            return ChatResponse(
                answer="",
                citations=[],
                confidence="low",
                session_id=session_id,
                error="The chatbot is temporarily unavailable. Please try again shortly.",
                error_code="GENERATION_ERROR",
            )

        # Step 6: Format citations
        citations = format_citations(chunks_used)

        # Step 7: Calculate confidence based on similarity scores
        avg_similarity = sum(c.similarity_score for c in chunks_used) / len(chunks_used)

        if avg_similarity >= 0.85:
            confidence = "high"
        elif avg_similarity >= 0.7:
            confidence = "medium"
        else:
            confidence = "low"

        # Step 8: Log query and response
        await log_query_and_response(
            query_id=query_id,
            session_id=session_id,
            query_text=query.query_text,
            selected_text=query.selected_text,
            answer=answer,
            token_count=token_count,
            confidence_score=avg_similarity,
            citations=citations,
        )

        # Step 9: Return response
        return ChatResponse(
            answer=answer,
            citations=citations,
            confidence=confidence,
            session_id=session_id,
        )

    except Exception as e:
        print(f"Unexpected error processing query: {e}")
        return ChatResponse(
            answer="",
            citations=[],
            confidence="low",
            session_id=query.session_id,
            error="An unexpected error occurred. Please try again.",
            error_code="INTERNAL_ERROR",
        )


async def log_query_and_response(
    query_id: str,
    session_id: str,
    query_text: str,
    selected_text: str,
    answer: str,
    token_count: int,
    confidence_score: float,
    citations: list = None,
):
    """
    Log query and response to database.

    Args:
        query_id: Unique query ID
        session_id: Session ID
        query_text: The user's query
        selected_text: Optional selected text
        answer: Generated answer
        token_count: Number of tokens in answer
        confidence_score: Average similarity score
        citations: List of citations
    """
    # Log query
    query_insert = """
        INSERT INTO queries (
            query_id,
            session_id,
            query_text,
            selected_text,
            timestamp
        ) VALUES ($1, $2, $3, $4, $5)
    """

    await execute_write(
        query_insert,
        query_id,
        session_id,
        query_text,
        selected_text,
        datetime.utcnow(),
    )

    # Log response
    source_references = None
    if citations:
        source_references = [
            {
                "section_title": c.section_title,
                "deep_link_url": c.deep_link_url,
                "chunk_count": c.chunk_count,
            }
            for c in citations
        ]

    response_insert = """
        INSERT INTO responses (
            response_id,
            query_id,
            response_text,
            token_count,
            source_references,
            confidence_score,
            generation_temperature,
            timestamp
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
    """

    temperature = float(os.getenv("TEMPERATURE_MIN", "0.3"))

    await execute_write(
        response_insert,
        str(uuid.uuid4()),
        query_id,
        answer,
        token_count,
        source_references,  # JSONB field
        confidence_score,
        temperature,
        datetime.utcnow(),
    )
