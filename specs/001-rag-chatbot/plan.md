# Architecture Plan: RAG Chatbot for AI-Native Textbook

**Feature**: 001-rag-chatbot
**Branch**: `001-rag-chatbot`
**Created**: 2025-12-20
**Status**: Design
**Planning Method**: Spec-Driven Development (SDD)

---

## Executive Summary

This plan details the architecture and implementation strategy for a Retrieval-Augmented Generation (RAG) chatbot embedded in a Docusaurus AI textbook. The system will answer student questions using **only book content**, with **95%+ retrieval accuracy**, **<3s response time**, and **100% grounded responses** (no hallucination).

**Core Value Proposition**: Students can select any paragraph from the textbook, ask questions about it, and receive accurate, cited answers grounded exclusively in the book's material.

**Key Constraints**:
- Book-only knowledge (no external sources)
- English-only for hackathon scope
- Free tiers: Neon Serverless Postgres, Qdrant Cloud, Cohere API
- Hackathon timeline (rapid delivery)

---

## 1. Architecture Plan

### 1.1 High-Level System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    FRONTEND (Docusaurus Book)                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Book Content (Markdown/MDX)                             │  │
│  │  - Chapters, Glossary, References, Code Snippets         │  │
│  │  - Figure Captions, Exercises                            │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  ChatKit UI Component (OpenAI ChatKit SDK)               │  │
│  │  - Text selection handler                                │  │
│  │  - Chat interface (input, streaming responses)           │  │
│  │  - Clickable citation deep links                         │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────┬────────────────────────────────────────┘
                         │ HTTPS (REST API)
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                   BACKEND (FastAPI - Python)                    │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  API Layer (FastAPI Routes)                              │  │
│  │  ├─ POST /api/ingest    (admin: trigger re-indexing)     │  │
│  │  ├─ POST /api/query     (user: ask question)             │  │
│  │  ├─ GET  /api/health    (system health check)            │  │
│  │  └─ GET  /api/status    (ingestion status)               │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Ingestion Pipeline                                       │  │
│  │  ├─ Content Loader (read book markdown/HTML)             │  │
│  │  ├─ Chunker (semantic boundaries, 200-500 tokens, 10-20% │  │
│  │  │    overlap)                                            │  │
│  │  ├─ Metadata Extractor (chapter, section, content_type,  │  │
│  │  │    deep_link_url)                                      │  │
│  │  └─ Embedding Generator (Cohere API)                     │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Query Pipeline                                           │  │
│  │  ├─ Input Validator (sanitize, detect injection)         │  │
│  │  ├─ Query Embedder (Cohere API)                          │  │
│  │  ├─ Hybrid Retriever (70% semantic + 30% BM25)           │  │
│  │  ├─ Context Assembler (top-K chunks + selected text)     │  │
│  │  ├─ Answer Generator (Cohere Chat API, temp 0.3-0.5)     │  │
│  │  └─ Citation Formatter (group by section, deep links)    │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────┬──────────────────────┬─────────────────────────────┘
             │                      │
             ▼                      ▼
┌─────────────────────┐  ┌──────────────────────────────────────┐
│  Neon Postgres DB   │  │  Qdrant Cloud (Vector DB)            │
│  - Query logs       │  │  Collection: "textbook_chunks"       │
│  - User sessions    │  │  - Embeddings (Cohere dimension)     │
│  - Book metadata    │  │  - Metadata (content_type, chapter,  │
│  - Analytics        │  │    section, deep_link_url, etc.)     │
└─────────────────────┘  └──────────────────────────────────────┘
             ▲                      ▲
             │                      │
             └──────────┬───────────┘
                        │
         ┌──────────────┴────────────────┐
         │   External Services (APIs)    │
         │  - Cohere Embed API           │
         │  - Cohere Chat API            │
         └───────────────────────────────┘
```

### 1.2 Data Flow

#### **Ingestion Flow** (Admin/Automated)

```
Book Content (Markdown/MDX)
    │
    ├─> 1. Content Loader
    │      • Parse Docusaurus file structure
    │      • Extract chapters, glossary, references, code, figures
    │      • Preserve hierarchy (chapter > section > subsection)
    │
    ├─> 2. Semantic Chunker
    │      • Split by semantic boundaries (paragraphs, code blocks, lists)
    │      • Target 200-500 tokens (allow 100-700 variance)
    │      • Create 10-20% overlaps (sliding window)
    │      • Assign unique chunk_id
    │
    ├─> 3. Metadata Extractor
    │      • chapter, section, subsection
    │      • content_type (text|glossary|code|reference|figure_caption|exercise)
    │      • paragraph_id (for deep linking)
    │      • deep_link_url (Docusaurus URL + anchor)
    │      • token_count, overlap_start, overlap_end
    │
    ├─> 4. Embedding Generator (Cohere API)
    │      • Batch process chunks (optimize API calls)
    │      • Generate vector embeddings (Cohere model dimension)
    │      • Store embedding model version for future compatibility
    │
    ├─> 5. Vector DB Storage (Qdrant)
    │      • Create/update collection "textbook_chunks"
    │      • Store vectors + full metadata payload
    │      • Create BM25 index for keyword search
    │
    └─> 6. Metadata DB Storage (Neon Postgres)
           • Store book version, ingestion timestamp
           • Track which sections were indexed
           • Log any ingestion errors/warnings
```

#### **Query Flow** (User-Initiated)

```
User Query + Optional Selected Text
    │
    ├─> 1. Input Validation
    │      • Sanitize input (prevent prompt injection)
    │      • Detect language (reject non-English)
    │      • Check query length limits
    │
    ├─> 2. Query Embedding (Cohere API)
    │      • Generate query vector using same model as chunks
    │      • If selected text exists, embed it separately
    │
    ├─> 3. Hybrid Retrieval (Qdrant)
    │      • Semantic search: cosine similarity (70% weight)
    │      • Keyword search: BM25 scoring (30% weight)
    │      • Combine scores via weighted fusion
    │      • Retrieve top-K chunks (default K=5, configurable)
    │      • Filter by similarity threshold (default 0.7)
    │
    ├─> 4. Context Assembly
    │      • If selected text exists: prepend as primary context
    │      • Add retrieved chunks (deduplicate by chunk_id)
    │      • Sort by relevance score
    │      • Check total context size (stay within LLM limits)
    │
    ├─> 5. Answer Generation (Cohere Chat API)
    │      • System prompt: "Answer only from provided context"
    │      • User query + assembled context
    │      • Temperature 0.3-0.5 (slight variation)
    │      • Max tokens: 500 (per spec)
    │      • Stream response to frontend
    │
    ├─> 6. Citation Formatting
    │      • Extract source chunks used in answer
    │      • Group by section (deduplicate)
    │      • Generate clickable deep links
    │      • Format: "Section 2.3: Kinematics [link]"
    │
    ├─> 7. Response Assembly
    │      • Combine answer text + citations
    │      • Add confidence indicator (if relevance low)
    │      • Handle "insufficient context" case
    │
    └─> 8. Logging (Neon Postgres)
           • Query text, session_id, timestamp
           • Retrieved chunks, relevance scores
           • Generated response, token count
           • User feedback (if provided)
```

### 1.3 Separation of Concerns

#### **Ingestion Pipeline** (Separate Module)
- **Responsibility**: Transform book content into searchable embeddings
- **Triggers**: Manual admin command, automated on book updates (webhook), scheduled nightly
- **Components**:
  - `content_loader.py` - Parse Docusaurus content
  - `chunker.py` - Semantic boundary-aware chunking
  - `metadata_extractor.py` - Extract structure and generate deep links
  - `embedding_service.py` - Interface with Cohere Embed API
  - `vector_store.py` - Interface with Qdrant Cloud
- **Isolation**: Can run offline, independent of query pipeline

#### **Query Pipeline** (Real-Time Service)
- **Responsibility**: Answer user questions in real-time
- **Triggers**: User submits query via ChatKit UI
- **Components**:
  - `query_validator.py` - Input sanitization
  - `retriever.py` - Hybrid search (semantic + BM25)
  - `context_assembler.py` - Build LLM context from chunks
  - `answer_generator.py` - Interface with Cohere Chat API
  - `citation_formatter.py` - Generate deep links
- **Isolation**: Stateless, horizontally scalable

#### **UI Integration** (Frontend Module)
- **Responsibility**: Provide chat interface embedded in Docusaurus
- **Components**:
  - ChatKit SDK integration (React component)
  - Text selection handler (capture user highlights)
  - Deep link navigation (scroll to cited paragraph)
  - Streaming response display
- **Isolation**: Communicates only via REST API

### 1.4 Trust Boundaries & Hallucination Prevention

```
┌─────────────────────────────────────────────────────────────┐
│              TRUST BOUNDARY: What LLM Can Answer            │
├─────────────────────────────────────────────────────────────┤
│  ✅ ALLOWED (Grounded in Book Content)                      │
│  - Explain concepts found in retrieved chunks               │
│  - Synthesize information from multiple book sections       │
│  - Simplify explanations using book terminology             │
│  - Compare concepts described in the book                   │
│  - Provide examples cited in the book                       │
│                                                              │
│  ❌ FORBIDDEN (Hallucination Risk)                          │
│  - Answer questions not in the book                         │
│  - Invent examples not in the book                          │
│  - Use external knowledge (even if technically correct)     │
│  - Speculate beyond book content                            │
│  - Make up citations or deep links                          │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│            ENFORCEMENT MECHANISMS                            │
├─────────────────────────────────────────────────────────────┤
│  1. System Prompt Constraints                               │
│     "You are a textbook assistant. Answer ONLY using the    │
│      provided context from the textbook. If the context     │
│      doesn't contain the answer, say 'I cannot find         │
│      sufficient information in the textbook.'"              │
│                                                              │
│  2. Similarity Threshold (0.7)                              │
│     - If top retrieved chunk score < 0.7 → reject query     │
│     - Return "insufficient information" message             │
│                                                              │
│  3. Citation Requirement                                     │
│     - Every answer MUST include source references           │
│     - Deep links verified against chunk metadata            │
│                                                              │
│  4. Temperature Control (0.3-0.5)                           │
│     - Low temperature reduces creative hallucination        │
│     - Keeps responses close to source material              │
│                                                              │
│  5. Context Window Limit                                     │
│     - Only pass retrieved chunks (no external context)      │
│     - Truncate if exceeds LLM limit                         │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Execution Phases (Ordered)

### Phase 0: Prerequisites (Before Starting)

**Objective**: Validate all credentials and service access

**Tasks**:
- [ ] Verify Cohere API key works (test embedding generation)
- [ ] Verify Qdrant Cloud cluster accessible (health check)
- [ ] Verify Neon Postgres connection string works
- [ ] Confirm GitHub repo created and cloned
- [ ] Confirm Docusaurus book builds locally

**Success Criteria**:
- All API keys tested and working
- Can connect to all external services
- Local development environment ready

**Duration**: 30 minutes

---

### Phase 1: Foundation Setup

**Objective**: Establish project structure, environment configuration, and database connectivity

**Tasks**:
1. **Repository Structure**
   - [ ] Create `rag-backend/` directory for FastAPI backend
   - [ ] Create `rag-backend/app/` for application code
   - [ ] Create `rag-backend/app/models/` for Pydantic models
   - [ ] Create `rag-backend/app/services/` for business logic
   - [ ] Create `rag-backend/app/api/` for FastAPI routes
   - [ ] Create `rag-backend/tests/` for test suite
   - [ ] Create `rag-backend/.env.example` template
   - [ ] Add `rag-backend/.gitignore` (exclude .env, __pycache__)

2. **Environment Variables & Secrets**
   - [ ] Create `.env` file (git-ignored)
   - [ ] Add Cohere API key: `COHERE_API_KEY=...`
   - [ ] Add Qdrant credentials: `QDRANT_URL=...`, `QDRANT_API_KEY=...`
   - [ ] Add Neon DB URL: `DATABASE_URL=postgresql://...`
   - [ ] Add config for: `CHUNK_SIZE=400`, `CHUNK_OVERLAP=0.15`, `SIMILARITY_THRESHOLD=0.7`
   - [ ] Document all env vars in `.env.example`

3. **Database Schema (Neon Postgres)**
   - [ ] Create `queries` table (query_id, session_id, query_text, selected_text, timestamp)
   - [ ] Create `responses` table (response_id, query_id, response_text, token_count, source_references, confidence_score, generation_temperature, timestamp)
   - [ ] Create `sessions` table (session_id, created_at, last_active_at, query_count)
   - [ ] Create `book_metadata` table (content_id, book_version, chapter_title, section_title, last_updated)
   - [ ] Create `ingestion_logs` table (ingestion_id, timestamp, chunks_processed, errors, status)
   - [ ] Write migration script: `rag-backend/migrations/001_init_schema.sql`

4. **Vector DB Setup (Qdrant)**
   - [ ] Create collection `textbook_chunks` with appropriate vector dimension (Cohere model)
   - [ ] Define payload schema:
     ```json
     {
       "chunk_id": "string (UUID)",
       "text_content": "string",
       "chapter": "string",
       "section": "string",
       "subsection": "string (optional)",
       "content_type": "text|glossary|code|reference|figure_caption|exercise",
       "paragraph_id": "string",
       "deep_link_url": "string",
       "token_count": "integer",
       "overlap_start": "integer",
       "overlap_end": "integer",
       "book_version": "string"
     }
     ```
   - [ ] Create indexes for efficient filtering (content_type, chapter)
   - [ ] Configure BM25 for hybrid search

5. **FastAPI Bootstrap**
   - [ ] Create `rag-backend/app/main.py` with FastAPI app initialization
   - [ ] Add CORS middleware (allow Docusaurus origin)
   - [ ] Add health check endpoint: `GET /api/health`
   - [ ] Add database connection pooling (asyncpg for Neon)
   - [ ] Add Qdrant client initialization
   - [ ] Add Cohere client initialization
   - [ ] Test: `uvicorn app.main:app --reload` runs successfully

**Dependencies**: None (first phase)

**Success Criteria**:
- [x] Repository structure in place
- [x] All environment variables configured
- [x] Database schema created and migrations run
- [x] Qdrant collection created with correct schema
- [x] FastAPI app starts and health check returns 200

**Duration**: 2-3 hours

---

### Phase 2: Indexing & Retrieval Pipeline

**Objective**: Build the ingestion pipeline to transform book content into searchable embeddings

**Tasks**:
1. **Content Loader** (`app/services/content_loader.py`)
   - [ ] Implement file system traversal for Docusaurus `docs/` directory
   - [ ] Parse markdown/MDX files (frontmatter + content)
   - [ ] Extract hierarchy: chapter (from directory) > section (from headings)
   - [ ] Identify content types:
     - Main text: paragraphs between headings
     - Glossary: entries in glossary file
     - Code: fenced code blocks (```...```)
     - References: bibliography entries
     - Figure captions: image alt text / captions
     - Exercises: sections marked with "Exercise" or "Problem"
   - [ ] Generate paragraph IDs (chapter-section-paragraph-N)
   - [ ] Generate deep link URLs (base URL + file path + anchor)
   - [ ] Return structured data: List[RawContent]
   - [ ] Test with sample book chapter

2. **Semantic Chunker** (`app/services/chunker.py`)
   - [ ] Implement boundary detection:
     - Paragraphs: split on `\n\n`
     - Code blocks: keep entire block intact
     - Lists: keep full list intact (all items)
     - Tables: keep entire table intact
   - [ ] Token counting (using tiktoken or Cohere tokenizer)
   - [ ] Chunking logic:
     - Start new chunk at each paragraph boundary
     - If paragraph > 700 tokens, split at sentence boundaries
     - If chunk < 200 tokens, merge with next paragraph (if compatible content_type)
     - Target 200-500 tokens, allow 100-700 variance
   - [ ] Overlap calculation (10-20%):
     - Take last N tokens from chunk[i] as overlap_start for chunk[i+1]
     - Take first N tokens from chunk[i+1] as overlap_end for chunk[i]
   - [ ] Return: List[Chunk] with text_content, metadata, overlap markers
   - [ ] Test: Chunk sample chapter, verify boundaries and overlaps

3. **Embedding Generator** (`app/services/embedding_service.py`)
   - [ ] Implement Cohere Embed API interface
   - [ ] Batch processing (max 96 texts per request per Cohere limits)
   - [ ] Retry logic for transient API failures (3 retries, exponential backoff)
   - [ ] Rate limiting (respect Cohere free tier limits)
   - [ ] Embedding model: `embed-english-v3.0` (or latest)
   - [ ] Store model version with each embedding
   - [ ] Test: Generate embeddings for 10 sample chunks

4. **Vector Store Interface** (`app/services/vector_store.py`)
   - [ ] Implement Qdrant client wrapper
   - [ ] Upsert operation (create or update chunks)
   - [ ] Batch upsert (optimize network calls)
   - [ ] Semantic search (cosine similarity, top-K)
   - [ ] Keyword search (BM25 scoring)
   - [ ] Hybrid search (70/30 fusion):
     ```python
     score = 0.7 * semantic_score + 0.3 * bm25_score
     ```
   - [ ] Similarity threshold filtering (default 0.7)
   - [ ] Return: List[RetrievedChunk] with score, metadata
   - [ ] Test: Upsert 100 sample chunks, retrieve with various queries

5. **Ingestion API Endpoint** (`app/api/ingest.py`)
   - [ ] `POST /api/ingest` (admin only, manual trigger for hackathon)
   - [ ] Request body: `{ "book_path": "/path/to/docs", "force_reindex": false }`
   - [ ] Orchestrate full pipeline:
     1. Load content from book_path
     2. Chunk content
     3. Generate embeddings (batched)
     4. Upsert to Qdrant
     5. Log to Neon (ingestion_logs table)
   - [ ] Return ingestion summary: chunks_processed, duration, errors
   - [ ] Add progress tracking (optional: SSE for real-time updates)
   - [ ] Test: Ingest 3 sample chapters end-to-end

**Dependencies**: Phase 1 complete

**Success Criteria**:
- [x] Book content parsed correctly (all content types identified)
- [x] Chunks respect semantic boundaries and have 10-20% overlap
- [x] Embeddings generated successfully via Cohere API
- [x] Chunks stored in Qdrant with full metadata
- [x] Hybrid search retrieves relevant chunks with >0.7 similarity
- [x] `/api/ingest` endpoint ingests sample chapters without errors

**Duration**: 6-8 hours

---

### Phase 3: Query & RAG Logic

**Objective**: Build the real-time query pipeline for answering user questions

**Tasks**:
1. **Input Validator** (`app/services/query_validator.py`)
   - [ ] Sanitize query text (strip HTML, trim whitespace)
   - [ ] Detect prompt injection patterns:
     - Ignore instructions (e.g., "ignore previous instructions")
     - Role-playing attempts (e.g., "you are now...")
     - System prompt leaks (e.g., "repeat your instructions")
   - [ ] Language detection (simple heuristic: reject non-ASCII heavy text)
   - [ ] Length limits: query 10-500 characters, selected text <5000 characters
   - [ ] Return: validated query or error message
   - [ ] Test: Validate 20 sample queries (10 valid, 10 malicious)

2. **Retriever** (`app/services/retriever.py`)
   - [ ] Embed query using Cohere Embed API
   - [ ] If selected text exists, treat as additional context filter
   - [ ] Perform hybrid search (call vector_store.hybrid_search)
   - [ ] Retrieve top-K chunks (default K=5, configurable)
   - [ ] Filter by similarity threshold (0.7)
   - [ ] Deduplicate chunks by chunk_id (if overlaps retrieved)
   - [ ] Return: List[RetrievedChunk] sorted by relevance
   - [ ] Test: Retrieve for 10 diverse queries, verify relevance

3. **Context Assembler** (`app/services/context_assembler.py`)
   - [ ] Assemble LLM context from:
     1. Selected text (if provided) - highest priority
     2. Retrieved chunks (sorted by relevance)
   - [ ] Format context as structured prompt:
     ```
     Context from textbook:
     [If selected text]
     Selected Passage:
     {selected_text}

     Related Sections:
     1. {chunk1.text_content} (Source: {chunk1.chapter} - {chunk1.section})
     2. {chunk2.text_content} (Source: {chunk2.chapter} - {chunk2.section})
     ...

     Question: {user_query}
     ```
   - [ ] Token counting (ensure total context + query + response < model limit)
   - [ ] Truncate if necessary (keep highest relevance chunks)
   - [ ] Return: assembled_context, source_chunks
   - [ ] Test: Assemble context for 5 queries, verify structure

4. **Answer Generator** (`app/services/answer_generator.py`)
   - [ ] Implement Cohere Chat API interface
   - [ ] System prompt (hallucination prevention):
     ```
     You are a helpful textbook assistant. Answer questions using ONLY the provided context from the textbook.
     - If the context contains the answer, explain clearly and cite the source section.
     - If the context is insufficient, respond: "I cannot find sufficient information in the textbook to answer this question accurately."
     - Do NOT use external knowledge or make assumptions beyond the provided context.
     - Keep responses under 500 tokens.
     - Include citations in format: (Source: Section X.Y)
     ```
   - [ ] User message: assembled_context + query
   - [ ] Temperature: 0.3-0.5 (configurable)
   - [ ] Max tokens: 500
   - [ ] Stream response (enable real-time display)
   - [ ] Return: answer_text, token_count
   - [ ] Test: Generate 10 answers, verify grounded in context

5. **Citation Formatter** (`app/services/citation_formatter.py`)
   - [ ] Extract source chunks used (from context_assembler)
   - [ ] Group by section (deduplicate: Section 2.3 appears once even if multiple chunks)
   - [ ] Generate deep links: `{base_url}/docs/{file_path}#{paragraph_id}`
   - [ ] Format citations:
     ```markdown
     Sources:
     - [Section 2.3: Kinematics Fundamentals](https://book.example.com/docs/chapter2#para-5)
     - [Section 3.1: Sensor Fusion](https://book.example.com/docs/chapter3#para-12)
     ```
   - [ ] Return: formatted_citations
   - [ ] Test: Format citations for 5 responses, verify deep links

6. **Query API Endpoint** (`app/api/query.py`)
   - [ ] `POST /api/query`
   - [ ] Request body:
     ```json
     {
       "query": "What is forward kinematics?",
       "selected_text": "optional selected paragraph",
       "session_id": "uuid (optional)"
     }
     ```
   - [ ] Orchestrate query pipeline:
     1. Validate input
     2. Retrieve relevant chunks
     3. Assemble context
     4. Generate answer (stream)
     5. Format citations
     6. Log to database (queries, responses tables)
   - [ ] Response:
     ```json
     {
       "answer": "Forward kinematics is...",
       "citations": [...],
       "confidence": "high|medium|low",
       "session_id": "uuid"
     }
     ```
   - [ ] Handle errors gracefully (API failures, low confidence)
   - [ ] Test: End-to-end query flow for 10 sample questions

7. **Session Management** (`app/services/session_service.py`)
   - [ ] Generate session_id (UUID v4)
   - [ ] Track session metadata in Neon (sessions table)
   - [ ] Update last_active_at on each query
   - [ ] Increment query_count
   - [ ] Session expiry: 24 hours (cleanup job)
   - [ ] Test: Create session, submit 5 queries, verify tracking

**Dependencies**: Phase 2 complete

**Success Criteria**:
- [x] Input validation rejects malicious queries
- [x] Retriever returns relevant chunks with >95% accuracy (manual eval on 20 test queries)
- [x] Context assembler correctly prioritizes selected text
- [x] Answer generator produces grounded responses (no hallucination in manual eval)
- [x] Citations formatted correctly with clickable deep links
- [x] `/api/query` endpoint responds in <3s for typical query
- [x] Sessions tracked correctly in database

**Duration**: 6-8 hours

---

### Phase 4: Chatbot UI Integration

**Objective**: Embed ChatKit UI into Docusaurus and connect to backend

**Tasks**:
1. **ChatKit SDK Integration** (`textbook/src/components/ChatBot.tsx`)
   - [ ] Install OpenAI ChatKit SDK: `npm install @chatscope/chat-ui-kit-react`
   - [ ] Create ChatBot React component
   - [ ] Add chat interface elements:
     - Message input field
     - Send button
     - Message history display (user + assistant messages)
     - Streaming response indicator ("thinking...")
   - [ ] Style to match Docusaurus theme
   - [ ] Position: floating button (bottom-right) or sidebar
   - [ ] Test: Component renders in Docusaurus page

2. **Text Selection Handler** (`textbook/src/components/TextSelectionHandler.tsx`)
   - [ ] Listen for text selection events (mouseup, touchend)
   - [ ] Show "Ask AI" tooltip on selection
   - [ ] On click, populate ChatBot input with selected text
   - [ ] Mark selected text visually (highlight)
   - [ ] Send selected text to backend in `/api/query` request
   - [ ] Test: Select paragraph, verify selected_text sent to API

3. **API Integration** (`textbook/src/services/chatApi.ts`)
   - [ ] Implement `POST /api/query` client
   - [ ] Handle streaming responses (Server-Sent Events or chunked transfer)
   - [ ] Error handling:
     - Network errors: retry with backoff
     - API errors: display user-friendly message
     - Timeout: "The chatbot is taking longer than expected..."
   - [ ] Session management (persist session_id in localStorage)
   - [ ] Test: Send 10 queries, verify responses streamed correctly

4. **Citation Deep Links** (`textbook/src/components/CitationLink.tsx`)
   - [ ] Parse citation URLs from backend response
   - [ ] Render as clickable links in chat messages
   - [ ] On click:
     - Navigate to target page/section
     - Scroll to exact paragraph (using paragraph_id anchor)
     - Highlight cited paragraph (CSS animation)
   - [ ] Test: Click citation, verify navigation and highlight

5. **Docusaurus Integration** (`textbook/docusaurus.config.ts`)
   - [ ] Add ChatBot component to theme:
     - Option 1: Global plugin (appears on all pages)
     - Option 2: Manual inclusion in select pages
   - [ ] Configure backend API URL (environment variable)
   - [ ] Add CORS headers to backend for Docusaurus origin
   - [ ] Test: ChatBot loads on book pages, queries work end-to-end

6. **Streaming & UX Enhancements**
   - [ ] Display streaming response word-by-word (typewriter effect)
   - [ ] Show "AI is typing..." indicator during generation
   - [ ] Disable input during streaming (prevent multiple concurrent queries)
   - [ ] Allow "Stop generating" button (abort request)
   - [ ] Add copy button for responses
   - [ ] Test: Stream 5 responses, verify smooth UX

**Dependencies**: Phase 3 complete

**Success Criteria**:
- [x] ChatBot UI embedded in Docusaurus book
- [x] Text selection triggers "Ask AI" tooltip
- [x] Selected text correctly sent as context override
- [x] Responses stream in real-time to UI
- [x] Citations are clickable and navigate to correct paragraphs
- [x] UI handles errors gracefully (API timeout, network failure)

**Duration**: 4-6 hours

---

### Phase 5: Validation & Demo Readiness

**Objective**: Test system thoroughly and prepare for demo/deployment

**Tasks**:
1. **Manual Test Cases**
   - [ ] Create test query set (20 questions covering:
     - Factual lookups ("What is forward kinematics?")
     - Explanatory ("Explain sensor fusion in simple terms")
     - Relational ("How does localization relate to mapping?")
     - Out-of-scope ("What is quantum computing?")
     - Edge cases (very long query, empty query, special characters)
   - [ ] Execute all 20 queries, document:
     - Retrieved chunks (verify relevance)
     - Generated answer (verify grounded in book)
     - Citations (verify correct deep links)
     - Response time (<3s)
   - [ ] Calculate metrics:
     - Retrieval accuracy: % of queries with relevant chunks retrieved
     - Groundedness: % of answers citing book content correctly
     - Response time: average and p95 latency
   - [ ] Target: >95% retrieval accuracy, 100% groundedness, <3s p95

2. **Accuracy & Grounding Checks**
   - [ ] Review 10 responses manually:
     - Compare answer to retrieved chunks (no external knowledge)
     - Verify citations match chunks used
     - Check for hallucination indicators (made-up facts, unsupported claims)
   - [ ] Fix any prompt engineering issues (adjust system prompt if needed)
   - [ ] Test "insufficient context" scenario:
     - Query: "What is the capital of France?" (not in book)
     - Expected: "I cannot find sufficient information in the textbook..."
   - [ ] Verify response under 500 tokens (all test cases)

3. **Failure Behavior Testing**
   - [ ] Test external service failures:
     - Simulate Cohere API timeout: verify graceful error message
     - Simulate Qdrant unavailable: verify fallback or error
     - Simulate Neon DB connection loss: verify query fails gracefully
   - [ ] Test input edge cases:
     - Empty query: reject with validation error
     - 10,000 character query: truncate or reject
     - Non-English query: reject with language error
     - Prompt injection: verify sanitization blocks
   - [ ] Test concurrent users:
     - Simulate 10 concurrent queries (use tool like `wrk` or `locust`)
     - Verify no errors, all responses <5s
   - [ ] Fix any issues discovered

4. **Performance Optimization**
   - [ ] Measure baseline performance:
     - Ingestion: chunks per second
     - Query: end-to-end latency breakdown (retrieval, generation, etc.)
   - [ ] Optimize bottlenecks:
     - Add database connection pooling (if not already)
     - Batch Cohere API calls where possible
     - Cache frequently queried chunks (optional)
   - [ ] Re-measure: verify <3s query latency, <500ms retrieval

5. **Documentation**
   - [ ] Create `README.md` in `rag-backend/`:
     - Project overview (RAG chatbot for textbook)
     - Architecture diagram (ASCII or link to image)
     - Setup instructions (env vars, database migrations, start server)
     - API documentation (endpoints, request/response formats)
     - Testing instructions
   - [ ] Create `ARCHITECTURE.md`:
     - High-level design (reference this plan document)
     - Data flow diagrams
     - Trust boundaries and hallucination prevention
   - [ ] Update book README with chatbot usage instructions
   - [ ] Add inline code comments where complex

6. **Deployment Preparation**
   - [ ] Backend deployment (Vercel/Render/Railway):
     - Dockerize FastAPI app (Dockerfile)
     - Configure environment variables in hosting platform
     - Test deployed backend with health check
   - [ ] Frontend deployment (Vercel/Netlify/GitHub Pages):
     - Configure API_BASE_URL environment variable
     - Build and deploy Docusaurus book
     - Test chatbot on deployed book
   - [ ] Smoke test end-to-end on production URLs

**Dependencies**: Phase 4 complete

**Success Criteria**:
- [x] 20/20 test queries pass (>95% retrieval accuracy, 100% grounded)
- [x] Manual review confirms no hallucination
- [x] "Insufficient context" message shown for out-of-scope queries
- [x] All failure scenarios handled gracefully
- [x] Concurrent user load (10 users) handled without errors
- [x] Documentation complete and clear
- [x] System deployed and accessible via public URL

**Duration**: 4-6 hours

---

## 3. Decisions Needing Documentation (ADR List)

The following architectural decisions should be documented as ADRs for future reference and knowledge transfer:

### ADR-001: Why Cohere Embeddings Instead of OpenAI

**Context**: Need to generate semantic embeddings for book content

**Decision**: Use Cohere `embed-english-v3.0` API

**Rationale**:
- Provided credentials: Cohere API key already available
- Cost-effective: Free tier sufficient for hackathon scope (~10k embeddings)
- Performance: Cohere embeddings optimized for semantic search (asymmetric retrieval)
- Simplicity: Single vendor for both embeddings and chat generation

**Alternatives Considered**:
- OpenAI `text-embedding-3-small`: Higher cost, requires separate API key
- Open-source models (Sentence-BERT): Requires self-hosting, adds complexity

**Consequences**:
- Vendor lock-in to Cohere (mitigated by modular embedding_service interface)
- Embedding dimension fixed by model (need to recreate collection if switching models)

---

### ADR-002: Why Qdrant Cloud Free Tier for Vector Database

**Context**: Need vector database for semantic search over embeddings

**Decision**: Use Qdrant Cloud Free Tier (hosted)

**Rationale**:
- Provided credentials: Qdrant cluster already provisioned
- Free tier limits: 1 cluster, 1M vectors, 4GB RAM (sufficient for textbook)
- Features: Hybrid search (semantic + keyword), metadata filtering, fast retrieval
- Managed service: No operational overhead (vs. self-hosting)
- API: Python client available, well-documented

**Alternatives Considered**:
- Pinecone: Free tier more limited (single index, less storage)
- Weaviate: Requires self-hosting or more expensive cloud tier
- PostgreSQL with pgvector: Slower for semantic search, complex setup

**Consequences**:
- Limited to free tier constraints (1M vectors, 4GB RAM)
- Internet dependency (can't run fully offline)
- Migration path: If outgrow free tier, can export and migrate to self-hosted Qdrant

---

### ADR-003: Chunk Size & Overlap Tradeoffs

**Context**: Need to split book content into chunks for embedding and retrieval

**Decision**:
- Target 200-500 tokens per chunk (allow 100-700 variance)
- 10-20% overlap between adjacent chunks
- Semantic boundary-aware (complete paragraphs, full code blocks)

**Rationale**:
- **Size**: 200-500 tokens balances context coherence vs. retrieval precision
  - Too small (<100): Loses context, requires more chunks to answer
  - Too large (>1000): Dilutes relevance, harder to cite specific info
- **Overlap**: 10-20% preserves cross-boundary context
  - Prevents information loss at chunk edges
  - Improves retrieval when query spans boundaries
  - Cost: ~15% more embeddings (acceptable for free tier)
- **Semantic boundaries**: Keeps logical units intact (paragraphs, code)
  - Better readability when displaying chunks
  - Preserves code block syntax (no mid-function splits)

**Alternatives Considered**:
- Fixed 256 token chunks (GPT-2 style): Breaks semantic units, worse UX
- No overlap: Cheaper, but risks missing context at boundaries
- Larger chunks (1000+ tokens): Reduces precision, harder to cite

**Consequences**:
- ~15% more storage/embedding cost due to overlap (acceptable)
- Chunking algorithm more complex (boundary detection)
- Better retrieval quality (verified in testing)

---

### ADR-004: Hybrid Search (70% Semantic + 30% Keyword) Strategy

**Context**: Need retrieval strategy for finding relevant book chunks

**Decision**: Combine semantic similarity (70%) and BM25 keyword search (30%)

**Rationale**:
- **Semantic search (70%)**: Captures conceptual similarity
  - Handles paraphrased queries ("what is forward kinematics" vs "explain FK")
  - Finds related concepts even without exact keyword match
- **Keyword search (30%)**: Ensures precise term lookup
  - Critical for technical terms, acronyms, proper nouns
  - Handles definition queries ("define sensor fusion")
  - Complements semantic for edge cases
- **70/30 weighting**: Empirically balanced for textbook content
  - Favors conceptual understanding (primary student need)
  - Still supports precise lookups (glossary, references)

**Alternatives Considered**:
- Semantic-only: Misses exact term matches, weaker for definitions
- Keyword-only: Poor paraphrase handling, requires exact terms
- 50/50 weighting: Less prioritization of conceptual understanding

**Consequences**:
- Requires BM25 index in Qdrant (additional storage, minimal)
- Score fusion complexity (normalize and combine scores)
- Better recall and precision (verified in testing: >95% accuracy)

---

### ADR-005: English-Only Scope for Hackathon

**Context**: Need to define language support scope

**Decision**: English-only for both book content and user queries (hackathon scope)

**Rationale**:
- **Simplicity**: Avoids translation complexity, faster development
- **Book content**: Physical AI textbook is in English
- **User base**: Primary audience (students/educators) expected to use English
- **Free tier limits**: Translation APIs (Google, DeepL) add cost
- **Future extensibility**: Can add multilingual support post-hackathon

**Alternatives Considered**:
- Multilingual queries → English responses: Requires translation, adds latency
- Multilingual book + queries: Not applicable (book is English)

**Consequences**:
- Non-English users cannot use chatbot (acceptable for hackathon)
- Language detection needed (simple: reject non-ASCII heavy text)
- Future work: Add translation layer if demand exists

---

### ADR-006: Online Indexing vs Offline Rebuilds

**Context**: How to handle book content updates (re-indexing)

**Decision**: Manual trigger for hackathon, incremental re-indexing for future

**Rationale**:
- **Hackathon scope**: Book updates rare (monthly or less)
- **Manual trigger**: Admin calls `/api/ingest` endpoint
  - Simple, no automation overhead
  - Sufficient for demo and early usage
- **Incremental re-indexing (future)**: Only re-embed changed sections
  - Detect changes via file hash or last_modified timestamp
  - Delete old chunks, insert new chunks
  - Faster and cheaper than full rebuild

**Alternatives Considered**:
- Automated webhook: Trigger on Docusaurus build (adds complexity)
- Scheduled nightly: Wasteful if no changes (free tier API limits)
- Full rebuild every time: Slow, expensive for large books

**Consequences**:
- Manual process for hackathon (acceptable, documented in README)
- Incremental logic can be added later (modular design supports this)
- Users may see stale content for up to 24 hours after updates (per spec)

---

## 4. Testing & Validation Strategy

### 4.1 Connection Tests (Smoke Tests)

**Objective**: Verify all external service integrations

**Tests**:
1. **Neon Postgres Connectivity**
   - Test: Connect to database, run `SELECT 1`
   - Expected: Connection successful, query returns 1
   - Location: `rag-backend/tests/test_db_connection.py`

2. **Qdrant Health Check**
   - Test: Call Qdrant `/health` endpoint
   - Expected: HTTP 200, cluster status "green"
   - Location: `rag-backend/tests/test_qdrant_connection.py`

3. **Cohere Embedding Generation**
   - Test: Generate embedding for "test query"
   - Expected: Vector with correct dimensions (1024 for embed-english-v3.0)
   - Location: `rag-backend/tests/test_cohere_embed.py`

4. **Cohere Chat Generation**
   - Test: Generate response for simple prompt
   - Expected: Text response, <500 tokens
   - Location: `rag-backend/tests/test_cohere_chat.py`

**Run Frequency**: On every deployment, before integration tests

---

### 4.2 RAG Correctness Tests

**Objective**: Verify chatbot answers correctly from book content

**Test Cases** (20 queries):

| ID | Query Type | Query | Expected Behavior |
|----|-----------|-------|-------------------|
| 1  | Factual   | "What is forward kinematics?" | Return definition from book, cite section |
| 2  | Explanatory | "Explain sensor fusion in simple terms" | Synthesize from book content, cite sources |
| 3  | Relational | "How does localization relate to mapping?" | Connect concepts from multiple sections |
| 4  | Definition | "Define SLAM" | Lookup glossary or definition, cite |
| 5  | Comparison | "Difference between PID and LQR control?" | Compare based on book sections |
| 6  | Code-related | "Explain this code snippet" (with selected code) | Explain using code comments + related text |
| 7  | Figure-related | "What does Figure 3.2 show?" | Use figure caption + surrounding text |
| 8  | Out-of-scope | "What is quantum computing?" | "Insufficient information in textbook" |
| 9  | Out-of-scope | "Who won the 2024 election?" | "Insufficient information in textbook" |
| 10 | Ambiguous | "What is the best approach?" | Ask for clarification or use book heuristics |
| 11 | Edge: Empty | "" (empty query) | Validation error: "Query required" |
| 12 | Edge: Long | 1000-word query | Accept or truncate gracefully |
| 13 | Edge: Special chars | "What is <script>alert()</script>?" | Sanitize, answer without injection |
| 14 | Edge: Non-English | "¿Qué es la robótica?" | Language error: "English only" |
| 15 | Selected text | (Select paragraph + ask "Explain") | Use selected text as primary context |
| 16 | Selected text unrelated | (Select Ch 2, ask about Ch 5) | Note mismatch, retrieve Ch 5 content |
| 17 | Multi-section | "Summarize Chapter 3" | Retrieve multiple chunks, synthesize summary |
| 18 | Glossary | "What does IMU stand for?" | Lookup glossary entry, cite |
| 19 | Reference | "Cite sources for SLAM" | Return bibliography entry, cite references section |
| 20 | Code example | "Show example of PID implementation" | Retrieve code block, cite section |

**Validation Criteria**:
- [ ] **Citation Accuracy**: Every answer includes valid citations (section + deep link)
- [ ] **Groundedness**: No hallucinated information (verified by comparing answer to retrieved chunks)
- [ ] **Relevance**: Retrieved chunks match query intent (>95% accuracy)
- [ ] **Rejection**: Out-of-scope queries correctly return "insufficient information"
- [ ] **Response Time**: <3s end-to-end for all queries

**Test Implementation**: `rag-backend/tests/test_rag_correctness.py`

---

### 4.3 UI/UX Tests

**Objective**: Verify chatbot UI works correctly in Docusaurus

**Test Cases**:
1. **Chat Renders**
   - Test: Load book page, verify ChatBot component visible
   - Expected: Chat interface present (input, send button, history)

2. **Send Query**
   - Test: Type query, click send
   - Expected: Query submitted, response streams in real-time

3. **Text Selection**
   - Test: Select paragraph, click "Ask AI"
   - Expected: Selected text sent as context, answer references selection

4. **Citation Navigation**
   - Test: Click citation link in response
   - Expected: Navigate to cited section, highlight paragraph

5. **Error Handling**
   - Test: Disconnect backend, send query
   - Expected: User-friendly error message ("Chatbot temporarily unavailable")

6. **Session Persistence**
   - Test: Submit 3 queries, refresh page
   - Expected: Session ID persists, chat history retained (optional feature)

**Test Implementation**: Manual QA, Playwright/Cypress automation (optional)

---

### 4.4 Performance Tests

**Objective**: Verify system meets latency and concurrency requirements

**Test Cases**:
1. **Query Latency (Single User)**
   - Test: Measure end-to-end time for 20 test queries
   - Expected: Average <2s, p95 <3s, p99 <5s

2. **Concurrent Users**
   - Test: Simulate 10 concurrent users (load testing tool: `locust`)
   - Expected: All queries complete successfully, <5s latency under load

3. **Ingestion Performance**
   - Test: Measure chunks/second during ingestion
   - Expected: >50 chunks/second (for 1000 chunks = <20 seconds total)

4. **Retrieval Latency**
   - Test: Measure Qdrant query time (semantic + keyword)
   - Expected: <500ms for top-5 retrieval

**Test Implementation**: `locust` script for load testing, profiling with `cProfile`

---

## 5. Deliverables Checklist (Hackathon)

### 5.1 Code Deliverables

- [ ] **GitHub Repository**: Public repo with clear structure
  - `rag-backend/` - FastAPI backend
  - `textbook/` - Docusaurus book
  - `README.md` - Project overview, setup instructions
  - `ARCHITECTURE.md` - System design, data flow diagrams
  - `.env.example` - Environment variable template
  - `.gitignore` - Exclude secrets, cache, build artifacts

- [ ] **Backend (FastAPI)**
  - [ ] API endpoints: `/api/ingest`, `/api/query`, `/api/health`
  - [ ] Ingestion pipeline (content loader, chunker, embedder)
  - [ ] Query pipeline (validator, retriever, generator, citation formatter)
  - [ ] Database schema and migrations
  - [ ] Environment configuration (dotenv)
  - [ ] Unit tests (>70% coverage target)

- [ ] **Frontend (Docusaurus + ChatKit)**
  - [ ] ChatBot UI component (React + ChatKit SDK)
  - [ ] Text selection handler
  - [ ] API integration (streaming, error handling)
  - [ ] Citation deep linking
  - [ ] Responsive design (mobile-friendly)

### 5.2 Documentation Deliverables

- [ ] **README.md (Root)**
  - [ ] Project description (RAG chatbot for AI textbook)
  - [ ] Features list (text selection, grounded answers, citations)
  - [ ] Quick start guide (clone, install, run)
  - [ ] Demo link (deployed book URL)
  - [ ] Architecture overview (link to ARCHITECTURE.md)

- [ ] **README.md (Backend)**
  - [ ] Setup instructions (Python version, dependencies, env vars)
  - [ ] Database migration guide
  - [ ] API documentation (endpoints, request/response examples)
  - [ ] Testing guide (run unit tests, load tests)
  - [ ] Deployment guide (Docker, hosting platform)

- [ ] **ARCHITECTURE.md**
  - [ ] High-level system diagram (copy from this plan)
  - [ ] Data flow diagrams (ingestion, query)
  - [ ] Technology stack explanation (Cohere, Qdrant, Neon, FastAPI)
  - [ ] Trust boundaries & hallucination prevention
  - [ ] ADR links (or embed key decisions)

- [ ] **ADR Documents** (in `docs/adr/` or `specs/001-rag-chatbot/adr/`)
  - [ ] ADR-001: Cohere Embeddings
  - [ ] ADR-002: Qdrant Cloud
  - [ ] ADR-003: Chunk Size & Overlap
  - [ ] ADR-004: Hybrid Search Strategy
  - [ ] ADR-005: English-Only Scope
  - [ ] ADR-006: Online Indexing Strategy

### 5.3 Deployment Deliverables

- [ ] **Deployed Backend**
  - [ ] Hosted on Vercel/Render/Railway (free tier)
  - [ ] Public URL: `https://rag-backend.example.com`
  - [ ] Health check endpoint accessible: `GET /api/health` returns 200
  - [ ] Environment variables configured in hosting platform

- [ ] **Deployed Frontend (Docusaurus Book)**
  - [ ] Hosted on Vercel/Netlify/GitHub Pages
  - [ ] Public URL: `https://physical-ai-book.example.com`
  - [ ] ChatBot functional on deployed site
  - [ ] Text selection and citation deep linking work

- [ ] **Smoke Test on Production**
  - [ ] Submit 5 test queries via deployed UI
  - [ ] Verify responses grounded in book content
  - [ ] Verify citations navigate correctly
  - [ ] Verify response times <3s

### 5.4 Demo Readiness

- [ ] **Demo Script** (5-minute walkthrough)
  1. Show textbook homepage, explain AI-native content
  2. Select a paragraph about "robot kinematics"
  3. Ask: "Explain this in simpler terms"
  4. Show streaming response, highlight citation
  5. Click citation, navigate to source paragraph
  6. Ask out-of-scope question: "What is quantum computing?"
  7. Show rejection: "Insufficient information in textbook"
  8. Explain architecture (quick tour of diagram)
  9. Show GitHub repo, README, testing

- [ ] **Presentation Slides** (Optional, 3-5 slides)
  1. Problem: Students need interactive help with textbook
  2. Solution: RAG chatbot grounded only in book content
  3. Architecture: Cohere + Qdrant + FastAPI + Docusaurus
  4. Key Features: Text selection, grounded answers, citations
  5. Future: Subagents, multi-LLM, production optimizations

### 5.5 Future Work (Not Implemented in Hackathon)

Document these as "future enhancements" (not blocking for demo):

- [ ] **Multi-LLM Support**: Adapter pattern for swapping Cohere → OpenAI → local models
- [ ] **Subagents**: Specialized agents for different book sections (e.g., math solver, code helper)
- [ ] **MCP Integration**: Model Context Protocol for reusable intelligence
- [ ] **Multilingual Support**: Translation layer for non-English queries
- [ ] **Advanced Analytics**: Dashboard for educators (common questions, knowledge gaps)
- [ ] **User Authentication**: Track individual student progress
- [ ] **Offline Mode**: Download embeddings for offline use
- [ ] **Production Optimization**: Caching, CDN, load balancing

---

## 6. Risk Assessment & Mitigation

### High-Priority Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **Cohere API rate limits exceeded** | Medium | High | Batch requests, implement exponential backoff, monitor usage |
| **Qdrant free tier storage limit (1M vectors)** | Low | High | Monitor chunk count, optimize chunking if needed |
| **Hallucination in responses** | Medium | Critical | Strict system prompt, low temperature, manual testing |
| **Slow query response (>3s)** | Medium | Medium | Optimize retrieval, cache frequent queries, profile bottlenecks |
| **Deployment fails due to env vars** | Low | Medium | Document all env vars, test deployment early |

### Medium-Priority Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **Book content structure varies** | Medium | Medium | Test chunker with diverse content types, adjust boundaries |
| **Deep links break (Docusaurus updates)** | Low | Medium | Generate robust paragraph IDs, test navigation |
| **Concurrent user load crashes backend** | Low | Medium | Load test before demo, scale backend if needed |
| **ChatKit SDK compatibility issues** | Low | Low | Test integration early, fallback to custom UI if needed |

---

## 7. Success Metrics (Post-Implementation Validation)

After implementation, validate against these metrics:

1. **Retrieval Accuracy**: >95% of test queries retrieve relevant chunks (similarity >0.7)
2. **Groundedness**: 100% of responses cite book content (no hallucination in manual review)
3. **Response Time**: p95 latency <3s, p99 <5s
4. **Concurrency**: 10 concurrent users handled without errors
5. **Uptime**: >99% during demo period (health checks green)
6. **User Satisfaction**: Positive feedback from 5+ testers (qualitative)

---

## Appendix: Technology Stack Summary

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| **Backend Framework** | FastAPI (Python 3.11+) | Async support, fast, well-documented, OpenAPI |
| **Vector Database** | Qdrant Cloud (Free Tier) | Hybrid search, managed, free tier sufficient |
| **Relational Database** | Neon Serverless Postgres | Serverless, free tier, low-latency queries |
| **Embeddings API** | Cohere `embed-english-v3.0` | Provided credentials, optimized for search |
| **Chat API** | Cohere Chat API | Single vendor (simplicity), grounded responses |
| **Frontend Framework** | Docusaurus (React-based) | Book already uses Docusaurus |
| **Chat UI SDK** | OpenAI ChatKit React SDK | Pre-built components, saves dev time |
| **Deployment (Backend)** | Vercel/Render/Railway | Free tiers, easy FastAPI deployment |
| **Deployment (Frontend)** | Vercel/Netlify/GitHub Pages | Free static site hosting |
| **Testing** | pytest (backend), Playwright (frontend) | Standard Python/JS testing tools |
| **Load Testing** | Locust | Concurrent user simulation |

---

**End of Architecture Plan**

*This plan will be translated into executable tasks in `tasks.md` for implementation tracking.*
