# ADR-001: Cohere Embeddings vs OpenAI

**Status**: Accepted
**Date**: 2025-12-20
**Deciders**: Architecture Team
**Feature**: 001-rag-chatbot

---

## Context

The RAG chatbot needs to generate semantic embeddings for textbook content to enable similarity-based retrieval. Embeddings are the foundation of the semantic search capability, which must achieve >95% retrieval accuracy per our success criteria.

**Key Requirements**:
- Generate high-quality embeddings for ~10,000 textbook chunks (chapters, glossary, references, code snippets)
- Optimize for asymmetric retrieval (user query → book content matching)
- Support English text only (hackathon scope)
- Work within free tier budget constraints
- Integrate seamlessly with chosen vector database (Qdrant)

**Constraints**:
- Hackathon timeline (rapid development, limited time for experimentation)
- Budget: Free tier APIs only
- Provided credentials: Cohere API key already available

---

## Decision

**We will use Cohere `embed-english-v3.0` API for generating all text embeddings.**

This decision applies to:
- Book content chunking (chapters, glossary, references, code, figure captions, exercises)
- User query embedding for similarity search
- All embedding generation in the ingestion pipeline

---

## Rationale

### Why Cohere

1. **Provided Credentials**: Cohere API key already available, eliminating setup friction

2. **Cost-Effective for Hackathon**:
   - Free tier: 100 API calls/minute, sufficient for batch processing ~10k chunks
   - Estimated cost: $0 (within free tier limits)
   - No credit card required for hackathon scope

3. **Optimized for Semantic Search**:
   - `embed-english-v3.0` specifically designed for asymmetric retrieval (query-to-document matching)
   - Better performance than general-purpose embeddings for RAG use cases
   - Compression-aware (works well with Qdrant's quantization)

4. **Single Vendor Simplicity**:
   - Using Cohere for both embeddings (Embed API) and chat generation (Chat API)
   - Reduces vendor fragmentation, simpler API key management
   - Consistent error handling and retry logic across services

5. **Proven Performance**:
   - Cohere embeddings rank highly on MTEB (Massive Text Embedding Benchmark)
   - Particularly strong for Q&A and information retrieval tasks
   - Handles technical/scientific text well (relevant for AI textbook)

### Tradeoffs Accepted

**Pros**:
- ✅ Zero cost for hackathon scope
- ✅ Fast setup (credentials provided)
- ✅ Optimized for our specific use case (asymmetric retrieval)
- ✅ Single vendor (Cohere) for embeddings + chat
- ✅ 1024-dimensional vectors (good balance of accuracy vs. storage)

**Cons**:
- ❌ Vendor lock-in: Changing embedding model requires re-embedding entire book (~10k chunks)
- ❌ API dependency: Requires internet connection, subject to rate limits
- ❌ Model updates: If Cohere deprecates `embed-english-v3.0`, migration needed
- ❌ Fixed dimensionality: 1024 dimensions locked once Qdrant collection created

---

## Alternatives Considered

### Alternative 1: OpenAI `text-embedding-3-small`

**Pros**:
- Industry-leading performance (slightly better on some benchmarks)
- More widely adopted (larger ecosystem, more examples)
- Ada-003 model optimized for search

**Cons**:
- **Cost**: OpenAI charges per token, estimated ~$2-5 for 10k chunks (budget constraint)
- **Separate API key**: Not provided, requires additional setup
- **Vendor fragmentation**: Would use OpenAI for embeddings + Cohere for chat (or switch both to OpenAI)
- **Higher dimensionality**: 1536 dimensions → more storage/compute in Qdrant

**Verdict**: Rejected due to cost and credential availability. Benefit doesn't justify hackathon setup overhead.

---

### Alternative 2: Open-Source Models (Sentence-BERT, BGE, etc.)

**Pros**:
- No API cost (run locally or self-hosted)
- No vendor lock-in
- Full control over model version
- Offline capability

**Cons**:
- **Self-hosting complexity**: Requires GPU for reasonable performance, adds infrastructure
- **Performance**: Many open-source models lag behind Cohere/OpenAI for semantic search
- **Integration effort**: Need to build embedding pipeline, no managed API
- **Hackathon timeline**: Too much setup time for uncertain quality gain

**Verdict**: Rejected due to complexity and time constraints. Not viable for hackathon delivery timeline.

---

### Alternative 3: Qdrant's Built-In Embedding Models

**Pros**:
- Integrated with vector database (fewer API calls)
- Potentially lower latency

**Cons**:
- Limited model selection
- Less control over embedding quality
- Less proven for our specific use case

**Verdict**: Rejected. Cohere's asymmetric retrieval optimization outweighs integration benefits.

---

## Consequences

### Positive Outcomes

1. **Rapid Development**: Zero setup time, immediate embedding generation
2. **Cost Savings**: Entirely free within hackathon scope (~10k embeddings)
3. **Proven Quality**: High confidence in retrieval accuracy (>95% target achievable)
4. **Simplified Stack**: Single vendor (Cohere) reduces integration complexity

### Negative Outcomes & Mitigations

1. **Vendor Lock-In**:
   - **Risk**: Switching away from Cohere requires re-embedding entire book
   - **Mitigation**: Modular `embedding_service.py` interface allows swapping implementations
   - **Future**: Adapter pattern supports OpenAI, open-source, or other providers

2. **API Rate Limits**:
   - **Risk**: Free tier limited to 100 calls/min; could bottleneck large re-indexing
   - **Mitigation**: Batch processing (96 texts per request), exponential backoff, chunked ingestion
   - **Future**: Upgrade to paid tier if needed (~$1/1M tokens)

3. **Embedding Dimension Lock-In**:
   - **Risk**: 1024-dim vectors locked once Qdrant collection created
   - **Mitigation**: Store model version with each embedding for auditability
   - **Future**: If switching models, create new Qdrant collection and migrate

4. **Internet Dependency**:
   - **Risk**: Cannot generate embeddings offline
   - **Mitigation**: Cache embeddings in Qdrant + Neon Postgres (redundancy)
   - **Future**: Export embeddings for offline backup

### Implementation Impact

**Files Affected**:
- `app/services/embedding_service.py`: Cohere API client, batch processing logic
- `app/models/chunk.py`: Embedding dimension (1024), model_version field
- `rag-backend/.env`: `COHERE_API_KEY` environment variable
- Qdrant collection schema: Vector dimension=1024, distance=Cosine

**Testing Requirements**:
- Unit test: Verify Cohere API connection, embedding generation for sample text
- Integration test: Embed 100 sample chunks, verify dimension and consistency
- Performance test: Measure embedding throughput (chunks/second)

---

## References

- **Plan Document**: `specs/001-rag-chatbot/plan.md` (Phase 2, Section 3: Embedding Generator)
- **Specification**: `specs/001-rag-chatbot/spec.md` (FR-002a: Generate semantic embeddings using Cohere API)
- **Constitution**: `.specify/memory/constitution.md` (Section V: Maintainability - modular design for swapping providers)
- **Cohere Docs**: https://docs.cohere.com/reference/embed
- **MTEB Leaderboard**: https://huggingface.co/spaces/mteb/leaderboard (Cohere v3 rankings)

---

## Acceptance Criteria

This ADR is accepted when:
- [x] Decision clusters related embedding choices (not atomic technology selection)
- [x] At least one alternative explicitly listed with rationale
- [x] Clear pros and cons for chosen approach and alternatives
- [x] Consequences cover both positive (rapid dev, cost savings) and negative (vendor lock-in, API limits) outcomes
- [x] References link back to plan, spec, and constitution
- [x] Implementation impact specified (files, testing)

---

## Review Notes

- **Analyzed**: Decision is architecturally significant (impacts how engineers structure embedding pipeline)
- **Measured**: PASS - Clusters embedding technology choice, lists 3 alternatives, documents tradeoffs comprehensively
- **Risk**: Medium - Vendor lock-in mitigated by modular interface design
- **Future Work**: Document migration path if switching to OpenAI/open-source in separate ADR when needed
