---
id: 001
title: Create RAG Chatbot Constitution
stage: constitution
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

/sp.constitution

Project: Integrated RAG Chatbot for AI-native textbook

Core principles:
- Accuracy: Chatbot must answer only from the book content or user-selected text
- Reliability: All embeddings and retrieval must reflect the most up-to-date book version
- Transparency: Responses should indicate if context is insufficient
- Security & privacy: No sensitive user data stored beyond session
- Maintainability: System architecture should allow future integration of new LLMs (like Cohere) or new book content

Key standards:
- Embeddings: Use Cohere API for generating text embeddings
- Vector database: Store and retrieve chunks in Qdrant Cloud Free Tier
- Backend: FastAPI must handle ingestion, retrieval, and query endpoints
- Database: Neon Serverless Postgres for metadata and user session tracking
- Chat UI: Integrated via OpenAI ChatKit SDK in Docusaurus book
- RAG behavior: Combine semantic + keyword search to retrieve top N relevant chunks
- User interaction: Chatbot only answers based on selected text or retrieved context; do not hallucinate
- Testing: All endpoints must be tested with sample book content before live deployment

Constraints:
- Cohere API key must be used, not OpenAI
- Responses must be under 500 tokens for quick UI display
- Only published book content can be referenced
- User-selected text overrides default retrieval context

Success criteria:
- Chatbot answers accurately from book content
- Semantic search retrieves relevant chunks 95%+ accuracy in test cases
- Backend endpoints function without errors and scale to multiple simultaneous users
- Chat UI correctly streams answers in the Docusaurus book interface
- System ready for future Subagents / reusable intelligence integration

## Response snapshot

Created comprehensive project constitution at `.specify/memory/constitution.md` with the following structure:

**Core Principles (6)**:
1. Accuracy - Grounded responses only from book content
2. Reliability - Up-to-date embeddings and 95%+ retrieval accuracy
3. Transparency - Clear communication about limitations
4. Security & Privacy - No sensitive data retention beyond session
5. Maintainability - Modular architecture for future LLM integration
6. Testability - All endpoints tested before deployment

**Technology Standards**:
- Stack: Cohere (embeddings), Qdrant (vector DB), FastAPI (backend), Neon Postgres (metadata), OpenAI ChatKit (UI)
- API Constraints: <500 tokens response, proper error handling, CORS configuration
- RAG Behavior: Semantic + keyword search, top N chunks (3-5 default), context-based generation

**Quality Gates**:
- Pre-deployment checklist (6 items)
- Performance benchmarks: <500ms retrieval, <3s end-to-end, 10+ concurrent users
- Code quality: Type hints, docstrings, linting

**Development Workflow**:
- Spec-first approach with plan → tasks → implement → test → document
- Testing strategy: unit, integration, manual QA, regression
- Version control: feature branches, PR reviews, conventional commits

**Constraints**:
- Hard: Cohere API only, <500 tokens, book content only, no sensitive data storage
- Soft: FastAPI features, Pydantic models, async/await, environment variables

**Future Extensibility**:
- Subagents/multi-agent workflows
- Multi-LLM support (Cohere, OpenAI, Anthropic, local)
- Multi-modal content (diagrams, code, equations)

## Outcome

- ✅ Impact: Established foundational governance for RAG chatbot backend project
- 🧪 Tests: No tests required for documentation
- 📁 Files: Created/updated `.specify/memory/constitution.md`
- 🔁 Next prompts: Create feature specs for core RAG pipeline components (ingestion, retrieval, chat)
- 🧠 Reflection: Constitution provides clear boundaries for accuracy, performance, and extensibility; enables consistent decision-making across team

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A - documentation artifact
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate constitution against existing codebase structure (if any) to ensure alignment
