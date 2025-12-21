---
id: 004
title: Document Cohere Embeddings ADR
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.adr
labels: ["adr", "embeddings", "cohere", "openai", "architecture"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend/history/adr/001-cohere-embeddings-vs-openai.md
  pr: null
files:
  - history/adr/001-cohere-embeddings-vs-openai.md
tests:
  - none
---

## Prompt

/sp.adr Cohere Embeddings vs OpenAI

## Response snapshot

**ADR-001 Created**: Cohere Embeddings vs OpenAI

**Workflow Executed**:
1. **Loaded Planning Context**: Found plan.md (56KB) and spec.md (17KB)
2. **Extracted Decision**: Found ADR-001 content in plan.md (lines 765-784)
3. **Checked Existing ADRs**: No conflicts, no existing ADR directory
4. **Applied Significance Test**: PASS
   - ✅ Impacts how engineers structure embedding pipeline (architecturally significant)
   - ✅ Notable tradeoffs: vendor lock-in vs. cost/simplicity
   - ✅ Will be questioned when scaling or switching providers
5. **Created ADR**: `history/adr/001-cohere-embeddings-vs-openai.md`

**ADR Content**:

**Context**:
- Need semantic embeddings for ~10k textbook chunks
- Requirements: >95% retrieval accuracy, free tier budget, English-only
- Constraints: Hackathon timeline, provided Cohere credentials

**Decision**:
- Use Cohere `embed-english-v3.0` API for all text embeddings
- Applies to: book content, user queries, ingestion pipeline

**Rationale** (5 points):
1. Provided credentials (Cohere API key available)
2. Cost-effective (free tier: 100 calls/min, sufficient for 10k chunks)
3. Optimized for semantic search (asymmetric retrieval for Q&A)
4. Single vendor simplicity (Cohere for embeddings + chat)
5. Proven performance (strong on MTEB for information retrieval)

**Tradeoffs**:
- **Pros**: Zero cost, fast setup, optimized for RAG, single vendor, 1024-dim vectors
- **Cons**: Vendor lock-in, API dependency, model update risk, fixed dimensionality

**Alternatives Considered** (3):
1. **OpenAI `text-embedding-3-small`**: Better performance, but $2-5 cost and separate API key (rejected: budget + setup overhead)
2. **Open-Source (Sentence-BERT)**: No cost, no lock-in, but self-hosting complexity and time (rejected: hackathon timeline)
3. **Qdrant Built-In**: Integrated, but limited selection and less proven (rejected: Cohere optimization superior)

**Consequences**:
- **Positive**: Rapid dev (zero setup), cost savings (free), proven quality (>95% target achievable), simplified stack
- **Negative**: Vendor lock-in (mitigated by modular interface), rate limits (mitigated by batching), dimension lock-in (store model version), internet dependency (cache embeddings)

**Implementation Impact**:
- Files: `embedding_service.py`, `chunk.py`, `.env`, Qdrant schema
- Testing: Unit (API connection), integration (100 chunks), performance (throughput)

**References**: plan.md (Phase 2), spec.md (FR-002a), constitution (maintainability), Cohere docs, MTEB leaderboard

## Outcome

- ✅ Impact: Formalized architectural decision on embedding technology; provides traceable rationale for future team members
- 🧪 Tests: Defined testing requirements (unit, integration, performance)
- 📁 Files: Created history/adr/001-cohere-embeddings-vs-openai.md (2.5KB)
- 🔁 Next prompts: Document remaining ADRs (Qdrant, chunking, hybrid search, English-only, indexing), then /sp.tasks
- 🧠 Reflection: ADR clusters embedding technology decision with clear alternatives, tradeoffs, and mitigation strategies. Significance test passed (impacts engineering structure, notable tradeoffs, revisitable).

## Evaluation notes (flywheel)

- Failure modes observed: None; decision already well-documented in plan.md
- Graders run and results (PASS/FAIL):
  - Analyze: PASS - Decision clusters embedding approach (not over-granular), lists 3 alternatives with rationale
  - Measure: PASS - Architecturally significant, explicit alternatives, clear pros/cons, concise yet detailed
- Prompt variant (if applicable): N/A - standard /sp.adr workflow
- Next experiment (smallest change to try): Create remaining 5 ADRs in batch, then validate consistency across all 6 ADRs before /sp.tasks
