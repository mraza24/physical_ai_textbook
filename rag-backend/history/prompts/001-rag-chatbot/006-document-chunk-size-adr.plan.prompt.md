---
id: 006
title: Document Chunk Size and Overlap Tradeoffs ADR
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.adr
labels: ["adr", "chunking", "semantic-boundaries", "overlap", "architecture"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend/history/adr/003-chunk-size-and-overlap-tradeoffs.md
  pr: null
files:
  - history/adr/003-chunk-size-and-overlap-tradeoffs.md
tests:
  - none
---

## Prompt

/sp.adr Chunk Size and Overlap Tradeoffs

## Response snapshot

**ADR-003 Created**: Chunk Size and Overlap Tradeoffs

**Decision**: Use semantic boundary-aware chunking with 200-500 token target, 100-700 token variance, 10-20% overlap

**Context**:
- Need chunking strategy for ~10k textbook chunks (main text, glossary, code, references, figures, exercises)
- Requirements: Context coherence, precise retrieval, accurate citations, >95% retrieval accuracy
- Challenges: Too small loses context, too large dilutes relevance, fixed boundaries break readability, no overlap loses boundary info

**Rationale** (3 components):
1. **200-500 Token Target with 100-700 Variance**: Balances context vs. precision, preserves semantic integrity over strict limits
   - Lower bound (200): Meaningful standalone chunks (paragraphs 100-200, glossary 50-150, code 100-300 tokens)
   - Upper bound (500): Prevents relevance dilution, precise citations (sections 300-600, long paragraphs 200-400 tokens)
   - Variance allows: Short glossary 80 tokens (acceptable), long code block 650 tokens (keep intact), paragraph with list 550 tokens (keep together)
   - Empirical support: 256-512 tokens optimal for Q&A retrieval (LangChain benchmarks)

2. **10-20% Overlap**: Cross-boundary context preservation
   - Problem: Questions spanning two chunks may not retrieve both
   - Solution: Last 50-100 tokens of chunk[i] = first 50-100 tokens of chunk[i+1]
   - Cost: 15% more vectors (10k → 11.5k), still within free tier (Qdrant 1M vectors, Cohere 100 calls/min)
   - Benefit: Improved recall, fewer missed relevant chunks

3. **Semantic Boundary Awareness**: Paragraphs, code blocks, lists, tables, sentences intact
   - Readability: Coherent text in UI, syntax highlighting works, all list items visible
   - Citation precision: Align with book structure (paragraph_id), deep links to exact paragraph
   - Retrieval quality: Semantic units are self-contained concepts, embedding captures complete thought

**Tradeoffs**:
- **Pros**: High retrieval quality (>95% target), precise citations, better UX (human-readable chunks), flexible adaptation (diverse content types)
- **Cons**: 15% higher storage/embedding cost (mitigated: free tier headroom), complex algorithm (mitigated: LangChain libraries), variable sizes (batch by token count), overlap deduplication needed (dedupe in context assembler)

**Alternatives Considered** (4):
1. **Fixed 256 Token Chunks**: Uniform sizing simplifies batching, but breaks semantic units, poor UX, citation imprecision, worse retrieval (rejected: quality degradation)
2. **No Overlap**: 15% savings, simpler logic, but boundary information loss, lower recall, no redundancy (rejected: cost savings not worth risk)
3. **Large Chunks 1000+**: Fewer chunks, more context per chunk, but diluted relevance, citation imprecision, poor UX (scroll fatigue), worse retrieval (rejected: conflicts with 95% accuracy)
4. **Sentence-Level Chunks**: Maximum granularity, boundaries respected, but too small (10-50 tokens), requires many chunks, higher latency, loses discourse structure (rejected: too granular)

**Consequences**:
- **Positive**: High retrieval quality (95%+ achievable), precise citations (deep links to paragraphs), better UX (coherent chunks), flexible adaptation (glossary/code/tables intact)
- **Negative**: 15% higher cost (mitigated: free tier 1M vectors), complex chunking (mitigated: LangChain RecursiveCharacterTextSplitter), variable sizes (batch optimization), overlap deduplication (dedupe by chunk_id)

**Implementation Impact**:
- Files: `chunker.py` (semantic boundary detection), `chunk.py` (`overlap_start`, `overlap_end` fields), `embedding_service.py` (batch variable sizes), `context_assembler.py` (deduplicate overlapping chunks)
- Chunking algorithm: Split on `\n\n` paragraphs, large paragraphs split at sentences, overlap_tokens = int(token_count * 0.15), sliding window
- Metadata schema: Added `overlap_start`, `overlap_end` to TextChunk (0 if first/last chunk)
- Testing: Unit (no mid-sentence splits, code blocks intact, overlap 10-20%, token variance 100-700), integration (chunk sample chapter, boundary-spanning retrieval), performance (chunks/second)

**Related ADRs**:
- ADR-001 (Cohere Embeddings): Chunk token count affects embedding API costs
- ADR-002 (Qdrant): Chunk metadata schema (overlap_start, overlap_end fields added to payload)

## Outcome

- ✅ Impact: Formalized chunking strategy; defines granularity and quality of semantic retrieval across entire RAG pipeline
- 🧪 Tests: Defined 3 test categories (unit: boundary integrity, integration: sample chapter, performance: throughput)
- 📁 Files: Created history/adr/003-chunk-size-and-overlap-tradeoffs.md (3.8KB)
- 🔁 Next prompts: Document remaining ADRs (hybrid search, English-only, indexing), then /sp.tasks
- 🧠 Reflection: ADR clusters chunking strategy (size + overlap + boundaries as integrated solution), with 4 alternatives and comprehensive cost/quality tradeoff analysis. Semantic boundary awareness critical for UX and citation precision.

## Evaluation notes (flywheel)

- Failure modes observed: None; decision well-documented in plan.md and clarification-analysis.md
- Graders run and results (PASS/FAIL):
  - Analyze: PASS - Clusters chunking strategy (not over-granular), lists 4 alternatives with detailed rationale
  - Measure: PASS - Architecturally significant (impacts retrieval quality, UX, cost across RAG pipeline), explicit alternatives, clear pros/cons, comprehensive detail with pseudocode
- Prompt variant (if applicable): N/A - standard /sp.adr workflow
- Next experiment (smallest change to try): Create remaining 3 ADRs (hybrid search, English-only, indexing), then validate all 6 ADRs for consistency before /sp.tasks
