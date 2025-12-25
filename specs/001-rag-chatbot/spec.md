# Feature Specification: RAG Chatbot for AI-Native Textbook

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-20
**Status**: Draft
**Input**: User description: "RAG Chatbot for AI-Native Book Project - Build a Retrieval-Augmented Generation (RAG) chatbot embedded in a Docusaurus book. Chatbot answers user questions based on selected text from the book. Backend powered by FastAPI, Neon Serverless Postgres, Qdrant Cloud, and Cohere embeddings."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Answer Questions from Book Content (Priority: P1)

A student reading the AI textbook in their browser encounters a complex concept and wants clarification. They select the relevant paragraph, click the chat interface, and ask "Can you explain this concept in simpler terms?" The chatbot reads the selected text, retrieves related content from the book, and provides a clear explanation grounded only in the book's material.

**Why this priority**: This is the core value proposition - accurate, contextual answers from book content. Without this, the chatbot has no purpose.

**Independent Test**: Can be fully tested by selecting any paragraph from the book, asking a question about it, and verifying the chatbot responds using only that paragraph and related book content.

**Acceptance Scenarios**:

1. **Given** a student has selected a paragraph about "robot kinematics", **When** they ask "What are the main types mentioned here?", **Then** the chatbot responds with types found in the selected paragraph and related book sections, citing the sources
2. **Given** a student asks a question without selecting text, **When** they submit "What is forward kinematics?", **Then** the chatbot searches the entire book for relevant content and answers based on the top matching sections
3. **Given** a student selects a paragraph about "sensor fusion", **When** they ask "How does this relate to localization?", **Then** the chatbot retrieves both the selected text and related localization content from the book to provide a comprehensive answer
4. **Given** insufficient content exists in the book to answer a question, **When** student asks "What is quantum computing?", **Then** the chatbot responds "I cannot find sufficient information about this topic in the textbook to provide an accurate answer"

---

### User Story 2 - Fast and Reliable Response Delivery (Priority: P2)

A student using the chatbot during study sessions expects quick, reliable answers without encountering errors or timeouts. When they ask questions, the system retrieves relevant content and generates responses within seconds, maintaining performance even during peak usage times (e.g., before exams when many students use the textbook simultaneously).

**Why this priority**: User experience depends on responsiveness. Slow or error-prone systems frustrate users and reduce adoption.

**Independent Test**: Can be tested by sending multiple concurrent queries and measuring response times and error rates under various load conditions.

**Acceptance Scenarios**:

1. **Given** a student asks a question, **When** the system processes the query, **Then** the chatbot returns a response within 3 seconds or shows a "still thinking" indicator
2. **Given** 10 students are using the chatbot simultaneously, **When** each submits a question, **Then** all receive responses without errors or timeouts
3. **Given** the backend services (vector database or LLM API) experience a temporary outage, **When** a student submits a question, **Then** the chatbot displays a friendly error message: "The chatbot is temporarily unavailable. Please try again shortly."
4. **Given** a student asks the same question twice, **When** the system processes both requests, **Then** both responses are semantically consistent (same core information, possibly different wording)

---

### User Story 3 - Transparent Context and Source Attribution (Priority: P3)

An educator reviewing the chatbot's answers wants to verify accuracy and understand which book sections informed each response. When the chatbot answers a student's question, it includes references to the specific book sections or chapters used, allowing both students and educators to trace the information back to its source.

**Why this priority**: Builds trust and supports academic integrity. Users can verify information and explore topics deeper by reading the source material.

**Independent Test**: Can be tested by asking questions and verifying that responses include clickable references or citations to specific book sections.

**Acceptance Scenarios**:

1. **Given** a student asks "What are the three types of robot joints?", **When** the chatbot responds, **Then** the answer includes references like "See Section 2.3: Kinematics Fundamentals" or similar source attribution
2. **Given** an educator reviews a chatbot response, **When** they click on a source reference, **Then** they are navigated to the exact book section that contributed to the answer
3. **Given** the chatbot uses information from multiple book sections, **When** it generates a response, **Then** all contributing sections are listed in the response
4. **Given** the chatbot's confidence in its answer is low (e.g., limited matching content found), **When** it responds, **Then** it explicitly states "Based on limited information in the textbook..." to indicate uncertainty

---

### Edge Cases

- **What happens when a student selects text but asks an unrelated question?**
  System should prioritize selected text as primary context but still use semantic search across the entire book if the question doesn't relate to the selection. Include a note in the response: "Your selected text doesn't directly address this question, but I found relevant information in [sections]."

- **How does the system handle very long selected text (e.g., entire chapter)?**
  System should chunk the selected text into manageable pieces and use semantic search within that chunk to find the most relevant subsections. Warn user if selection is too large: "Your selection is quite long. For better results, try selecting a specific paragraph or section."

- **What happens when the book content is updated after embeddings are generated?**
  System must detect stale embeddings and trigger re-ingestion. Display a banner to users: "The textbook was recently updated. Some answers may not reflect the latest content until re-indexing completes."

- **How does the system handle malformed or malicious queries (e.g., prompt injection attempts)?**
  System should sanitize inputs and detect prompt injection patterns. Respond with: "I can only answer questions about the textbook content. Please rephrase your question."

- **What happens when embeddings API (Cohere) or vector database (Qdrant) is unavailable?**
  System should gracefully degrade: display an error message to users and log the incident. For queries, attempt to use keyword-based fallback search if available; otherwise, return: "The chatbot is temporarily unavailable due to a service outage."

- **How does the system handle questions in languages other than the book's language?**
  For hackathon scope, the system only accepts English queries. Non-English queries should be detected and respond with: "I can only answer questions in English. Please rephrase your question in English."

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST embed all published book content including main chapter text, glossary entries, references, figure captions, code snippets/comments, and exercises as text chunks with associated metadata (chapter, section, page reference, content_type)
- **FR-002**: System MUST chunk content using semantic boundary-aware strategy (complete paragraphs, full code blocks, intact list items) with 200-500 token target size (allowing 100-700 variance for coherence) and 10-20% overlap between adjacent chunks
- **FR-002a**: System MUST generate semantic embeddings for each text chunk using Cohere API
- **FR-003**: System MUST store embeddings and chunk metadata in Qdrant vector database for efficient retrieval
- **FR-004**: System MUST accept user queries as plain text input through a chat interface
- **FR-005**: System MUST accept user-selected text from the book as optional context for queries
- **FR-006**: System MUST perform hybrid search combining semantic similarity (70% weight via Cohere embeddings) and BM25 keyword matching (30% weight) for all queries, with or without user-selected text as context
- **FR-007**: System MUST perform semantic search across all book content when no text is selected
- **FR-008**: System MUST retrieve the top N most relevant text chunks (N configurable, default 3-5) based on similarity scores
- **FR-009**: System MUST generate answers using Cohere Chat API (temperature 0.3-0.5) with retrieved chunks as context, using hybrid approach: extract facts directly from chunks, synthesize explanations in natural language, ensuring all responses are grounded only in book content
- **FR-010**: System MUST return responses under 500 tokens to ensure quick UI display
- **FR-011**: System MUST include clickable deep links to exact Docusaurus paragraphs as source references for all information used in responses, grouping multiple chunks from the same section into one section-level citation
- **FR-012**: System MUST detect when retrieved content has insufficient relevance (similarity score below threshold) and respond with "I cannot find sufficient information in the textbook to answer this question accurately"
- **FR-013**: System MUST log all queries and responses with metadata (timestamp, user session ID, retrieved chunks, response) in Neon Postgres for analytics
- **FR-014**: System MUST validate and sanitize user inputs to prevent prompt injection or malicious queries
- **FR-015**: System MUST handle concurrent requests from multiple users without degradation or errors
- **FR-016**: System MUST expose RESTful API endpoints for: content ingestion, query processing, and health checks
- **FR-017**: System MUST integrate chat UI into Docusaurus book interface using OpenAI ChatKit SDK
- **FR-018**: System MUST stream responses to the chat UI to provide real-time feedback to users
- **FR-019**: System MUST detect when book content has been updated and trigger re-ingestion of affected sections
- **FR-020**: System MUST handle API failures (Cohere, Qdrant) gracefully with user-friendly error messages

### Key Entities

- **TextChunk**: A segment of book content (semantically-aware chunks, 200-500 tokens target with 10-20% overlap) with associated metadata
  - Attributes: chunk_id, text_content, chapter, section, page_reference, paragraph_id, content_type (text|glossary|code|reference|figure_caption|exercise), deep_link_url, token_count, overlap_start, overlap_end
  - Relationships: Has one embedding vector; belongs to one book section

- **Embedding**: Vector representation of a text chunk's semantic meaning
  - Attributes: embedding_id, chunk_id, vector (dimensions based on Cohere model), model_version, created_at
  - Relationships: Associated with one text chunk

- **Query**: A user's question or request for information
  - Attributes: query_id, session_id, query_text, selected_text (optional), timestamp
  - Relationships: Has one response; retrieves multiple text chunks

- **Response**: The chatbot's answer to a query
  - Attributes: response_id, query_id, response_text, token_count, source_references (deep_link_urls grouped by section), confidence_score, generation_temperature, timestamp
  - Relationships: Answers one query; cites multiple text chunks (grouped by section)

- **UserSession**: Temporary session tracking for analytics (no personal data stored)
  - Attributes: session_id, created_at, last_active_at, query_count
  - Relationships: Has multiple queries

- **BookContent**: Metadata about the textbook's structure and content
  - Attributes: content_id, book_version, chapter_title, section_title, last_updated
  - Relationships: Contains multiple text chunks

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can receive accurate answers to questions about book content with 95%+ semantic relevance (measured by retrieval accuracy on test query set)
- **SC-002**: Chatbot responds to user queries within 3 seconds end-to-end (including retrieval, generation, and streaming)
- **SC-003**: System maintains <500ms retrieval latency for top 5 relevant chunks from vector database
- **SC-004**: System handles 10+ concurrent users submitting queries simultaneously without errors or degradation
- **SC-005**: 100% of chatbot responses are grounded in book content with source attribution (no hallucinated information)
- **SC-006**: Users can successfully select text and ask related questions 100% of the time without UI errors
- **SC-007**: Chat interface correctly streams responses in the Docusaurus book with proper formatting and references
- **SC-008**: System correctly identifies insufficient context scenarios and responds transparently at least 90% of the time
- **SC-009**: All API endpoints return appropriate HTTP status codes and error messages (no unhandled exceptions)
- **SC-010**: System successfully reflects updated book content in search results within 24 hours of content changes

## Assumptions

1. **Book Content Format**: Assuming book content is available as structured markdown or HTML with clear chapter/section hierarchy for chunking
2. **Language**: Assuming book is in English and queries will be in English (see edge case clarification above)
3. **Authentication**: Assuming no user authentication required for hackathon scope (all users anonymous with session tracking only)
4. **Content Updates**: Assuming book content updates are infrequent (monthly or less) during hackathon period
5. **User Base**: Assuming target users are students and educators with basic familiarity with chatbot interfaces
6. **Deployment**: Assuming deployment on standard cloud infrastructure (not production-scale optimizations)
7. **LLM for Generation**: Using Cohere API for both embeddings and chat response generation to simplify API integration and align with provided credentials
8. **Similarity Threshold**: Assuming a default similarity score threshold of 0.7 (on 0-1 scale) for determining relevance (adjustable based on testing)

## Architectural Decisions (from /sp.clarify)

The following critical decisions were clarified to inform architecture planning:

9. **Content Indexing Scope**: Index all book content types including main chapter text, glossary, references, figure captions, code comments, and exercises. Different content types (glossary entries, code snippets, diagrams) will be tagged with special metadata in Qdrant for potential filtered retrieval.

10. **Chunking Strategy**: Use semantic boundary-aware chunking (complete paragraphs, full code blocks, intact list items) with target size of 200-500 tokens but allow variance (100-700 tokens) to preserve coherence. Chunks will overlap by 10-20% (sliding window) to preserve cross-boundary context.

11. **Hybrid Search Implementation**: Combine semantic similarity (70% weight) and BM25 keyword matching (30% weight) for retrieval. This favors conceptual understanding while still supporting precise term lookup.

12. **Citation Format**: Responses will include clickable deep links to exact Docusaurus paragraphs where information was found. Multiple chunks from the same section will be grouped into one section-level citation for clarity.

13. **Response Generation Style**: Use Cohere Chat API with temperature=0.3-0.5 for slight natural variation while maintaining consistency. Responses will use a hybrid approach: extract facts directly from retrieved chunks, but synthesize explanations in natural language to improve readability.

## Constraints

### Hard Constraints
- Backend framework: FastAPI (Python)
- Relational database: Neon Serverless Postgres
- Vector database: Qdrant Cloud Free Tier
- Embeddings API: Cohere (not OpenAI)
- Chat UI integration: OpenAI ChatKit SDK in Docusaurus
- Response length: Maximum 500 tokens per response
- Timeline: Complete within hackathon period
- Data retention: No sensitive user data stored beyond session scope

### Soft Constraints
- Prefer async/await for I/O operations in FastAPI
- Prefer Pydantic models for request/response validation
- Prefer environment variables for API keys and database credentials
- Prefer storing raw text chunks in Postgres alongside metadata for redundancy
- Prefer incremental re-indexing (detect and re-embed only changed sections) over full re-indexing when book content updates

## Out of Scope

- User authentication and authorization (all users anonymous for hackathon)
- Multi-language support for queries or responses
- Integration with other LLM providers beyond Cohere (planned for future)
- Production-scale optimizations (caching, CDN, load balancing)
- Mobile-specific UI optimizations
- Offline mode or progressive web app features
- Non-book-related chat features (general conversation, jokes, etc.)
- Real-time collaborative features (multiple users discussing same content)
- Advanced analytics dashboard for educators
- Export/download of chat history
- Integration with learning management systems (LMS)
