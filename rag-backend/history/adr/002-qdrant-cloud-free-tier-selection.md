# ADR-002: Qdrant Cloud Free Tier for Vector Database

**Status**: Accepted
**Date**: 2025-12-20
**Deciders**: Architecture Team
**Feature**: 001-rag-chatbot

---

## Context

The RAG chatbot requires a vector database to store and retrieve semantic embeddings efficiently. The vector database is critical infrastructure that must support:

**Key Requirements**:
- Store ~10,000 embedding vectors (1024 dimensions each, from Cohere `embed-english-v3.0`)
- Perform fast similarity search (cosine distance, <500ms retrieval latency for top-5 results)
- Support hybrid search (semantic similarity + BM25 keyword matching at 70/30 weighting)
- Handle concurrent queries from multiple users (10+ simultaneous)
- Support metadata filtering (by content_type, chapter, section)
- Integrate with Python backend (FastAPI)

**Constraints**:
- Hackathon budget: Free tier only
- Provided credentials: Qdrant Cloud cluster already provisioned
- Timeline: Minimal setup time, focus on RAG logic implementation
- Scalability: Must handle textbook growth (potentially 50k+ chunks in future)

---

## Decision

**We will use Qdrant Cloud Free Tier (hosted managed service) as the vector database.**

This decision includes:
- **Deployment**: Qdrant Cloud hosted (managed service, no self-hosting)
- **Tier**: Free tier (1 cluster, 1M vectors, 4GB RAM)
- **Collection Design**: Single collection `textbook_chunks` with payload schema for metadata
- **Search Strategy**: Hybrid search (semantic cosine similarity + BM25 keyword matching)
- **Client**: Python `qdrant-client` library for FastAPI integration

---

## Rationale

### Why Qdrant Cloud Free Tier

1. **Provided Credentials & Zero Setup**:
   - Qdrant Cloud cluster already provisioned with API key
   - Eliminates infrastructure setup time (vs. self-hosting)
   - Immediate availability for development

2. **Free Tier Sufficient for Hackathon Scope**:
   - **Capacity**: 1M vectors, 4GB RAM → supports 10k chunks (~1024 dimensions) with room to grow
   - **Performance**: Managed infrastructure optimized for low-latency retrieval
   - **Cost**: $0 for hackathon scope, no credit card required

3. **Hybrid Search Native Support**:
   - Built-in cosine similarity search for semantic retrieval
   - BM25 keyword search for term-based matching
   - Score fusion at query time (70% semantic / 30% keyword per spec)
   - Critical for achieving >95% retrieval accuracy target

4. **Rich Metadata Filtering**:
   - Payload schema supports complex metadata (content_type, chapter, section, deep_link_url)
   - Filter by content_type (e.g., glossary-only search for definitions)
   - Filter by chapter for scoped retrieval

5. **Managed Service Benefits**:
   - No operational overhead (backups, updates, scaling handled by Qdrant)
   - High availability (no single point of failure management)
   - Monitoring and logging included

6. **Python Client Maturity**:
   - Official `qdrant-client` library well-documented
   - Async support (works seamlessly with FastAPI async endpoints)
   - Batch operations (optimize network calls for ingestion)

### Tradeoffs Accepted

**Pros**:
- ✅ Zero infrastructure management (no Docker, no Kubernetes, no server provisioning)
- ✅ Free for hackathon scope (1M vectors > 10k needed)
- ✅ Hybrid search native (semantic + keyword without custom logic)
- ✅ Fast setup (cluster already provisioned)
- ✅ Metadata filtering (content_type, chapter-level scoping)
- ✅ Scalability path (upgrade to paid tier or self-host if outgrow free tier)

**Cons**:
- ❌ Free tier limits: 1M vectors, 4GB RAM (constrains maximum book size)
- ❌ Internet dependency (cannot run fully offline during development)
- ❌ Vendor lock-in (migrating to another vector DB requires re-indexing)
- ❌ Less control vs. self-hosting (can't tune underlying config, e.g., HNSW parameters)

---

## Alternatives Considered

### Alternative 1: Pinecone

**Pros**:
- Industry-leading vector database (widely adopted)
- Simple API (fewer concepts than Qdrant)
- Serverless architecture (auto-scaling)

**Cons**:
- **Free tier more limited**: Single index, 1GB storage (vs. Qdrant 4GB)
- **No native BM25**: Hybrid search requires custom implementation
- **No provided credentials**: Would need separate sign-up and API key
- **Higher cost at scale**: Paid tiers more expensive than Qdrant

**Verdict**: Rejected due to weaker free tier (1GB vs. 4GB), lack of native BM25, and no provided credentials.

---

### Alternative 2: Weaviate

**Pros**:
- Strong hybrid search capabilities
- GraphQL API (flexible querying)
- Multi-modal support (images, text)

**Cons**:
- **Self-hosting required**: Cloud offering expensive (starts at $25/mo)
- **Complex setup**: Requires Docker/Kubernetes for local dev
- **Hackathon timeline**: Too much infrastructure overhead
- **No provided credentials**: Additional setup friction

**Verdict**: Rejected due to self-hosting complexity and lack of affordable managed tier for hackathon.

---

### Alternative 3: PostgreSQL with pgvector Extension

**Pros**:
- Already using Neon Postgres (single database for vectors + metadata)
- No separate service to manage
- Familiar SQL interface
- Open-source (no vendor lock-in)

**Cons**:
- **Performance**: Slower than specialized vector DBs (pgvector uses exact search, not HNSW)
- **No native BM25**: Would need separate full-text search implementation
- **Complex indexing**: Requires manual index tuning (IVFFlat, HNSW via extensions)
- **Less proven**: Newer extension, less mature ecosystem than Qdrant

**Verdict**: Rejected due to performance concerns (<500ms retrieval latency target) and lack of native hybrid search.

---

### Alternative 4: Self-Hosted Qdrant (Docker)

**Pros**:
- Full control over configuration (HNSW parameters, quantization)
- No free tier limits (only constrained by hardware)
- Offline capability (run locally without internet)
- Same Qdrant API as Cloud (easy migration path)

**Cons**:
- **Infrastructure overhead**: Requires Docker, persistent storage, backups
- **Hackathon timeline**: Setup time diverted from RAG logic
- **Operational burden**: Need to monitor, update, scale manually
- **Deployment complexity**: More moving parts in production

**Verdict**: Rejected due to operational complexity and hackathon timeline. Self-hosting viable for post-hackathon production if free tier outgrown.

---

## Consequences

### Positive Outcomes

1. **Rapid Development**: Zero infrastructure setup, immediate embedding storage
2. **Performance**: <500ms retrieval latency achievable with managed optimization
3. **Hybrid Search**: Native semantic + BM25 supports >95% retrieval accuracy target
4. **Scalability Path**: Can upgrade to paid tier ($95/mo for 10M vectors) or self-host if needed
5. **Maintainability**: Managed service reduces operational burden during/after hackathon

### Negative Outcomes & Mitigations

1. **Free Tier Capacity Limit (1M vectors)**:
   - **Risk**: Textbook growth beyond 10k chunks could exhaust free tier
   - **Mitigation**: Monitor vector count, optimize chunking strategy to reduce redundancy
   - **Future**: Upgrade to Qdrant Cloud paid tier or migrate to self-hosted Qdrant

2. **Internet Dependency**:
   - **Risk**: Cannot develop fully offline, subject to network availability
   - **Mitigation**: Cache frequently queried results in Neon Postgres, use local Qdrant for dev (optional)
   - **Future**: Export vectors for offline backup

3. **Vendor Lock-In**:
   - **Risk**: Switching to another vector DB (Pinecone, Weaviate) requires re-indexing all embeddings
   - **Mitigation**: Modular `vector_store.py` interface abstracts Qdrant-specific calls
   - **Future**: Qdrant supports export/import, can migrate to self-hosted or other DBs

4. **Limited Configuration Control**:
   - **Risk**: Cannot tune HNSW parameters (ef_construct, M) for optimal performance
   - **Mitigation**: Qdrant Cloud defaults are well-tuned for most use cases
   - **Future**: Self-host Qdrant if need custom tuning (e.g., extreme latency requirements)

### Implementation Impact

**Files Affected**:
- `app/services/vector_store.py`: Qdrant client wrapper, hybrid search implementation
- `app/models/chunk.py`: Payload schema matching Qdrant collection
- `rag-backend/.env`: `QDRANT_URL`, `QDRANT_API_KEY` environment variables
- Phase 1 tasks: Qdrant collection creation (`textbook_chunks`, 1024 dimensions, cosine distance)

**Collection Schema**:
```python
{
  "collection_name": "textbook_chunks",
  "vector_size": 1024,  # Cohere embed-english-v3.0 dimension
  "distance": "Cosine",
  "payload_schema": {
    "chunk_id": "keyword",
    "text_content": "text",
    "chapter": "keyword",
    "section": "keyword",
    "content_type": "keyword",  # text|glossary|code|reference|figure_caption|exercise
    "paragraph_id": "keyword",
    "deep_link_url": "keyword",
    "token_count": "integer",
    "overlap_start": "integer",
    "overlap_end": "integer",
    "book_version": "keyword"
  },
  "hnsw_config": {  # Managed by Qdrant Cloud
    "m": 16,        # Default, good balance
    "ef_construct": 100
  }
}
```

**Hybrid Search Implementation**:
```python
# 70% semantic + 30% BM25
semantic_results = qdrant.search(
    collection_name="textbook_chunks",
    query_vector=query_embedding,
    limit=10
)
bm25_results = qdrant.search(
    collection_name="textbook_chunks",
    query_text=query_text,  # BM25 search on text_content
    using="text_content",
    limit=10
)
# Fuse scores: 0.7 * semantic_score + 0.3 * bm25_score
```

**Testing Requirements**:
- **Unit Test**: Qdrant health check (`/health` endpoint), connection validation
- **Integration Test**: Upsert 100 sample chunks, retrieve with various queries, verify hybrid search
- **Performance Test**: Measure retrieval latency (<500ms target for top-5)
- **Load Test**: Simulate 10 concurrent queries, verify no degradation

---

## References

- **Plan Document**: `specs/001-rag-chatbot/plan.md` (Section 1.1: High-Level System Architecture, Phase 2: Vector Store Interface)
- **Specification**: `specs/001-rag-chatbot/spec.md` (FR-003: Store embeddings in Qdrant, FR-006: Hybrid search)
- **Constitution**: `.specify/memory/constitution.md` (Section II: Reliability - 95%+ retrieval accuracy)
- **Clarification Analysis**: `specs/001-rag-chatbot/analysis/clarification-analysis.md` (Q3: Hybrid Search Implementation)
- **Related ADR**: ADR-001 (Cohere Embeddings) - Embedding dimension (1024) determines Qdrant vector size
- **Qdrant Docs**: https://qdrant.tech/documentation/cloud/
- **Qdrant Python Client**: https://github.com/qdrant/qdrant-client

---

## Acceptance Criteria

This ADR is accepted when:
- [x] Decision clusters vector database selection (storage + search + deployment)
- [x] At least one alternative explicitly listed with rationale (4 alternatives documented)
- [x] Clear pros and cons for chosen approach and alternatives
- [x] Consequences cover both positive (rapid dev, performance) and negative (limits, lock-in) outcomes
- [x] References link back to plan, spec, constitution, and related ADRs
- [x] Implementation impact specified (collection schema, hybrid search code, testing)

---

## Review Notes

- **Analyzed**: Decision is architecturally significant (defines vector storage/retrieval layer structure)
- **Measured**: PASS - Clusters vector database choice (storage, search, deployment as integrated solution), lists 4 alternatives, documents tradeoffs comprehensively
- **Risk**: Medium - Free tier limits and vendor lock-in mitigated by modular interface and export capability
- **Dependency**: Requires ADR-001 (Cohere Embeddings) - vector dimension (1024) must match
- **Future Work**: If outgrow free tier, document migration to paid tier or self-hosted Qdrant in new ADR
