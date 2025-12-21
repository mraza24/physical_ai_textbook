---
id: 005
title: Document Qdrant Cloud Free Tier ADR
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.adr
labels: ["adr", "qdrant", "vector-database", "hybrid-search", "architecture"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend/history/adr/002-qdrant-cloud-free-tier-selection.md
  pr: null
files:
  - history/adr/002-qdrant-cloud-free-tier-selection.md
tests:
  - none
---

## Prompt

/sp.adr Qdrant Cloud Free Tier Selection

## Response snapshot

**ADR-002 Created**: Qdrant Cloud Free Tier for Vector Database

**Decision**: Use Qdrant Cloud Free Tier (hosted managed service) as vector database

**Context**:
- Need vector DB for ~10k embeddings (1024 dim from Cohere)
- Requirements: <500ms retrieval, hybrid search (semantic + BM25), concurrent users (10+), metadata filtering
- Constraints: Free tier budget, provided Qdrant credentials, hackathon timeline

**Rationale** (6 points):
1. Provided credentials (Qdrant cluster already provisioned, zero setup)
2. Free tier sufficient (1M vectors, 4GB RAM > 10k chunks needed)
3. Hybrid search native (semantic cosine + BM25 keyword matching)
4. Rich metadata filtering (content_type, chapter, section)
5. Managed service (no operational overhead, high availability)
6. Python client maturity (async support, batch operations)

**Tradeoffs**:
- **Pros**: Zero infrastructure mgmt, free for hackathon, hybrid search native, fast setup, metadata filtering, scalability path
- **Cons**: Free tier limits (1M vectors, 4GB), internet dependency, vendor lock-in, less control vs self-hosting

**Alternatives Considered** (4):
1. **Pinecone**: Better brand, but weaker free tier (1GB vs 4GB), no native BM25, no provided credentials (rejected: capacity + hybrid search)
2. **Weaviate**: Strong hybrid search, but requires self-hosting or expensive cloud ($25/mo), complex setup (rejected: complexity + cost)
3. **PostgreSQL + pgvector**: Single DB for vectors + metadata, familiar SQL, but slower performance, no native BM25 (rejected: <500ms latency target)
4. **Self-Hosted Qdrant**: Full control, no limits, offline, but infrastructure overhead, ops burden (rejected: hackathon timeline)

**Consequences**:
- **Positive**: Rapid dev (zero setup), performance (<500ms achievable), hybrid search (>95% accuracy), scalability (upgrade path), maintainability (managed)
- **Negative**: Free tier capacity (mitigated by monitoring, optimization), internet dependency (cache in Postgres), vendor lock-in (modular interface), limited config control (defaults well-tuned)

**Implementation Impact**:
- Files: `vector_store.py`, `chunk.py`, `.env` (QDRANT_URL, QDRANT_API_KEY)
- Collection schema: `textbook_chunks` (1024 dim, cosine distance, payload with content_type/chapter/deep_link_url)
- Hybrid search code: 70% semantic + 30% BM25 score fusion
- Testing: Unit (health check), integration (100 chunks), performance (<500ms), load (10 concurrent)

**Related ADRs**:
- ADR-001 (Cohere Embeddings): Vector dimension (1024) must match Qdrant collection

## Outcome

- ✅ Impact: Formalized vector database architecture; defines storage/retrieval layer for entire RAG system
- 🧪 Tests: Defined 4 test categories (unit, integration, performance, load)
- 📁 Files: Created history/adr/002-qdrant-cloud-free-tier-selection.md (3.2KB)
- 🔁 Next prompts: Document remaining ADRs (chunking, hybrid search, English-only, indexing), then /sp.tasks
- 🧠 Reflection: ADR clusters vector database decision (storage + search + deployment), with 4 alternatives and comprehensive tradeoff analysis. Hybrid search native support critical for achieving >95% retrieval accuracy target.

## Evaluation notes (flywheel)

- Failure modes observed: None; decision well-documented in plan.md
- Graders run and results (PASS/FAIL):
  - Analyze: PASS - Clusters vector DB selection (not over-granular), lists 4 alternatives with detailed rationale
  - Measure: PASS - Architecturally significant (defines entire vector storage layer), explicit alternatives, clear pros/cons, comprehensive detail
- Prompt variant (if applicable): N/A - standard /sp.adr workflow
- Next experiment (smallest change to try): Create remaining 4 ADRs, then validate all 6 ADRs for consistency before /sp.tasks
