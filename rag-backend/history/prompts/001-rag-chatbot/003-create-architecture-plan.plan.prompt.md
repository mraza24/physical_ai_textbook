---
id: 003
title: Create Architecture Plan for RAG Chatbot
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.plan
labels: ["plan", "architecture", "rag", "phases", "adr", "testing"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/plan.md
tests:
  - none
---

## Prompt

/sp.plan

Project: RAG Chatbot for an AI-Native Book (Hackathon Core Deliverable)

Inputs considered:
- Constitution: Accuracy, book-grounded answers only, RAG-first architecture
- Specification: FastAPI + Qdrant + Neon + Cohere embeddings, English-only
- Clarifications: Academic book Q&A, English-only, online indexing, living book

Create the following planning artifacts:

1. Architecture Plan
   - High-level system architecture (frontend, backend, data stores)
   - Data flow:
     Book content → chunking → embeddings (Cohere) → Qdrant
     User query / selected text → retriever → LLM → grounded answer
   - Separation of concerns:
     • Ingestion pipeline
     • Query pipeline
     • UI integration (ChatKit)
   - Explicitly show trust boundaries (what LLM can and cannot answer)

2. Execution Phases (Ordered)
   Phase 1: Foundation
   Phase 2: Indexing & Retrieval
   Phase 3: Query & RAG Logic
   Phase 4: Chatbot Interface
   Phase 5: Validation & Demo Readiness

3. Decisions Needing Documentation (ADR list)
4. Testing & Validation Strategy
5. Deliverables Checklist (Hackathon)

Constraints:
- No vibe coding: all implementation must follow specs & plan
- English-only for hackathon scope
- Book-only knowledge (no external web search)
- Free tiers only (Neon, Qdrant, Cohere)

## Response snapshot

**Comprehensive Architecture Plan Created**: 700+ line plan.md document with 7 major sections

**1. Architecture Plan** (Diagrams + Design):
- **High-Level System Architecture**: ASCII diagram showing Frontend (Docusaurus + ChatKit) → Backend (FastAPI) → Data Stores (Neon Postgres + Qdrant) → External Services (Cohere)
- **Ingestion Flow**: 6-step pipeline (Content Loader → Chunker → Metadata Extractor → Embedding Generator → Vector DB Storage → Metadata DB Storage)
- **Query Flow**: 8-step pipeline (Input Validation → Query Embedding → Hybrid Retrieval → Context Assembly → Answer Generation → Citation Formatting → Response Assembly → Logging)
- **Separation of Concerns**: 3 independent modules (Ingestion Pipeline, Query Pipeline, UI Integration) with clear component breakdown
- **Trust Boundaries**: Explicit definition of what LLM can/cannot answer, 5 enforcement mechanisms (system prompt, similarity threshold, citation requirement, temperature control, context window limit)

**2. Execution Phases** (6 phases, dependency-ordered):
- **Phase 0: Prerequisites** (30 min) - Validate all credentials and service access
- **Phase 1: Foundation** (2-3 hrs) - Repo structure, env vars, DB schema, FastAPI bootstrap
- **Phase 2: Indexing & Retrieval** (6-8 hrs) - Content loader, chunker, embedder, vector store, ingestion API
- **Phase 3: Query & RAG Logic** (6-8 hrs) - Validator, retriever, context assembler, answer generator, citation formatter, query API
- **Phase 4: Chatbot UI** (4-6 hrs) - ChatKit integration, text selection, API client, deep links, streaming
- **Phase 5: Validation & Demo** (4-6 hrs) - 20 test queries, accuracy checks, failure testing, docs, deployment

**3. ADR List** (6 architectural decisions documented):
- ADR-001: Why Cohere Embeddings Instead of OpenAI
- ADR-002: Why Qdrant Cloud Free Tier
- ADR-003: Chunk Size & Overlap Tradeoffs
- ADR-004: Hybrid Search (70% Semantic + 30% Keyword) Strategy
- ADR-005: English-Only Scope for Hackathon
- ADR-006: Online Indexing vs Offline Rebuilds

Each ADR includes: Context, Decision, Rationale, Alternatives Considered, Consequences

**4. Testing & Validation Strategy** (4 test categories):
- **Connection Tests**: Neon, Qdrant, Cohere Embed, Cohere Chat (smoke tests)
- **RAG Correctness Tests**: 20 test queries (factual, explanatory, relational, out-of-scope, edge cases) with validation criteria (citation accuracy, groundedness, relevance, rejection, response time)
- **UI/UX Tests**: 6 manual test cases (chat renders, send query, text selection, citation navigation, error handling, session persistence)
- **Performance Tests**: Query latency, concurrent users (10), ingestion throughput, retrieval latency

**5. Deliverables Checklist** (5 categories):
- **Code**: GitHub repo, backend (FastAPI), frontend (Docusaurus + ChatKit), tests
- **Documentation**: README (root + backend), ARCHITECTURE.md, 6 ADRs
- **Deployment**: Backend (Vercel/Render/Railway), Frontend (Vercel/Netlify/GitHub Pages), smoke test
- **Demo Readiness**: 5-minute demo script, presentation slides (optional), future work documented
- **Future Work**: Multi-LLM, Subagents, MCP, multilingual, analytics, auth, offline, production optimizations (not in hackathon scope)

**6. Risk Assessment**:
- High-priority: Cohere rate limits, Qdrant storage, hallucination, slow queries, deployment issues
- Medium-priority: Content structure variance, deep link breaks, concurrent load, ChatKit compatibility
- Mitigation strategies documented for each

**7. Success Metrics**:
- Retrieval accuracy >95%
- Groundedness 100%
- Response time p95 <3s
- Concurrency 10 users
- Uptime >99%
- User satisfaction (5+ testers)

**Technology Stack Table**: 13 components with rationale (FastAPI, Qdrant, Neon, Cohere, Docusaurus, ChatKit, pytest, Locust, etc.)

## Outcome

- ✅ Impact: Complete architectural blueprint ready for implementation; all phases, tasks, and decisions documented
- 🧪 Tests: Defined 20 RAG correctness tests, 4 connection tests, 6 UI tests, 4 performance tests
- 📁 Files: plan.md (700+ lines, 7 major sections)
- 🔁 Next prompts: `/sp.tasks` to break down phases into executable tasks, or `/sp.adr <title>` to document specific architectural decisions
- 🧠 Reflection: Plan provides clear "what comes first/next/last" execution order, trust boundaries for hallucination prevention, and complete testing strategy. All decisions traced back to constitution (accuracy, reliability, transparency) and spec clarifications (chunking, hybrid search, citations).

## Evaluation notes (flywheel)

- Failure modes observed: None; plan comprehensively addresses all requirements from prompt
- Graders run and results (PASS/FAIL): Plan validation - PASS (all 5 required artifacts present, phases dependency-ordered, ADRs documented, testing strategy complete, deliverables checklist actionable)
- Prompt variant (if applicable): Standard /sp.plan workflow with enhanced detail for ADRs and testing
- Next experiment (smallest change to try): Use this plan to generate tasks.md and validate that all implementation steps are covered without gaps
