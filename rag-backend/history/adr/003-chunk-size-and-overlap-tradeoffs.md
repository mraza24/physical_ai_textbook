# ADR-003: Chunk Size and Overlap Tradeoffs

**Status**: Accepted
**Date**: 2025-12-20
**Deciders**: Architecture Team
**Feature**: 001-rag-chatbot

---

## Context

The RAG chatbot must split textbook content into chunks for embedding generation and semantic retrieval. Chunking strategy directly impacts retrieval quality, citation precision, and user experience.

**Key Requirements**:
- Maintain context coherence (chunks should be self-contained and meaningful)
- Enable precise retrieval (chunks should be granular enough to pinpoint specific information)
- Support accurate citations (chunks should align with readable book sections/paragraphs)
- Optimize for retrieval accuracy (>95% target per spec)
- Work within embedding budget (Cohere free tier: 100 calls/min)

**Challenges**:
- **Too small**: Chunks <100 tokens lose context, require multiple chunks to answer questions
- **Too large**: Chunks >1000 tokens dilute relevance, make citation imprecise, harder to rank
- **Fixed boundaries**: Splitting mid-sentence or mid-code-block breaks readability
- **No overlap**: Information spanning chunk boundaries may be lost during retrieval

**Content Types to Chunk**:
- Main chapter text (paragraphs, sentences)
- Glossary entries (term + definition)
- Code snippets (functions, examples)
- References (bibliography entries)
- Figure captions (image descriptions)
- Exercises (problems + solutions)

---

## Decision

**We will use semantic boundary-aware chunking with the following parameters:**

1. **Target Size**: 200-500 tokens per chunk (flexible, not strict)
2. **Variance Allowed**: 100-700 tokens (to preserve semantic boundaries)
3. **Overlap**: 10-20% of chunk content overlaps with adjacent chunks (sliding window)
4. **Boundary Awareness**: Chunks respect semantic units:
   - **Paragraphs**: Keep complete paragraphs intact (split on `\n\n`)
   - **Code blocks**: Keep entire code block intact (no mid-function splits)
   - **Lists**: Keep full lists intact (all items together)
   - **Tables**: Keep entire table intact
   - **Sentences**: If paragraph >700 tokens, split at sentence boundaries

---

## Rationale

### Why 200-500 Token Target

**Balance of Context vs. Precision**:
- **Lower bound (200 tokens)**: Ensures chunk contains enough context to be meaningful standalone
  - Typical paragraph: 100-200 tokens
  - Short glossary entry: 50-150 tokens
  - Code snippet: 100-300 tokens
- **Upper bound (500 tokens)**: Prevents dilution of relevance, keeps citations precise
  - Typical section subsection: 300-600 tokens
  - Long paragraph: 200-400 tokens
  - Complex code example: 400-600 tokens

**Empirical Support**:
- Research shows 256-512 token chunks optimal for Q&A retrieval (LangChain benchmarks)
- Larger chunks (1000+) reduce precision in semantic search (more irrelevant content per chunk)
- Smaller chunks (<100) require more chunks per query, increasing latency and cost

**Variance Justification (100-700 tokens)**:
- Preserves semantic integrity over strict token limits
- Examples:
  - Short glossary entry: 80 tokens → acceptable (100-700 range)
  - Long code block: 650 tokens → keep intact (better than splitting mid-function)
  - Paragraph with embedded list: 550 tokens → keep together (better UX)

### Why 10-20% Overlap

**Cross-Boundary Context Preservation**:
- **Problem**: Questions spanning two chunks may not retrieve both if no overlap
  - Example: "How does X relate to Y?" where X is end of chunk[i], Y is start of chunk[i+1]
- **Solution**: 10-20% overlap ensures boundary information appears in both chunks
  - Last 50-100 tokens of chunk[i] = first 50-100 tokens of chunk[i+1]
  - Increases likelihood of retrieving complete context

**Cost Analysis**:
- **Storage cost**: ~15% more vectors in Qdrant (10k chunks → 11.5k with overlap)
- **Embedding cost**: ~15% more API calls to Cohere (still within free tier: 100 calls/min)
- **Benefit**: Improved retrieval recall (fewer missed relevant chunks)

**Alternative overlap strategies rejected**:
- **No overlap**: Cheapest, but risks missing context at boundaries (worse retrieval)
- **50% overlap**: Better context, but 2x storage/embedding cost (overkill)
- **Fixed token overlap** (e.g., always 50 tokens): Doesn't account for chunk size variance

### Why Semantic Boundary Awareness

**Readability & UX**:
- **Complete paragraphs**: User sees coherent text when chunk displayed in UI
- **Intact code blocks**: Syntax highlighting works, code is runnable/understandable
- **Full lists**: All items visible, no "item 3 of unknown total"

**Citation Precision**:
- Chunks align with natural book structure (paragraph_id, code_block_id)
- Deep links navigate to exact paragraph, not mid-sentence

**Retrieval Quality**:
- Semantic units (paragraphs, code blocks) are self-contained concepts
- Embedding captures complete thought, not fragment

---

## Alternatives Considered

### Alternative 1: Fixed 256 Token Chunks (GPT-2 Style)

**Approach**: Split content into uniform 256-token chunks, regardless of boundaries

**Pros**:
- Uniform chunk sizes simplify batching and indexing
- Predictable embedding costs (fixed number of chunks per content length)
- Easier to implement (no boundary detection logic)

**Cons**:
- ❌ **Breaks semantic units**: May split mid-sentence, mid-paragraph, mid-code-block
- ❌ **Poor UX**: Chunks not human-readable when displayed
- ❌ **Citation imprecision**: Cannot map to book paragraphs/sections cleanly
- ❌ **Worse retrieval**: Fragments lose context, require more chunks to answer

**Verdict**: Rejected. UX and retrieval quality degradation outweighs implementation simplicity.

---

### Alternative 2: No Overlap (Strictly Non-Overlapping Chunks)

**Approach**: Each chunk starts exactly where previous ends, no overlap

**Pros**:
- ✅ Lower storage cost (~15% savings vs. 10-20% overlap)
- ✅ Lower embedding cost (~15% fewer API calls)
- ✅ Simpler chunking logic (no overlap calculation)

**Cons**:
- ❌ **Boundary information loss**: Questions spanning two chunks may miss one chunk in retrieval
- ❌ **Lower recall**: Risk of missing relevant context at chunk edges
- ❌ **No redundancy**: If one chunk scores slightly below threshold, no second chance via overlapping chunk

**Verdict**: Rejected. 15% cost savings not worth retrieval quality risk. Free tier has plenty of headroom (1M vectors vs. 11.5k needed).

---

### Alternative 3: Large Chunks (1000+ Tokens)

**Approach**: Use 1000-1500 token chunks to maximize context per chunk

**Pros**:
- ✅ Fewer total chunks (lower storage/embedding cost)
- ✅ More context per retrieved chunk (less need to retrieve multiple chunks)

**Cons**:
- ❌ **Diluted relevance**: Large chunks contain more irrelevant info, harder to rank precisely
- ❌ **Citation imprecision**: Citing 1000-token chunk doesn't pinpoint specific info
- ❌ **Poor UX**: Too much text to display in chat UI (scroll fatigue)
- ❌ **Worse retrieval**: Semantic search scores less discriminative (all chunks have mixed content)

**Verdict**: Rejected. Retrieval precision and UX suffer. Conflicts with >95% accuracy target.

---

### Alternative 4: Sentence-Level Chunks

**Approach**: Each chunk = single sentence or 2-3 sentences

**Pros**:
- ✅ Maximum granularity (most precise citations)
- ✅ Semantic boundaries naturally respected

**Cons**:
- ❌ **Too small**: Sentences lack context (10-50 tokens each)
- ❌ **Requires many chunks**: Need 5-10 chunks to answer typical question
- ❌ **Higher latency**: More chunks to retrieve and fuse
- ❌ **Loss of discourse structure**: Paragraphs build arguments across sentences

**Verdict**: Rejected. Too granular, loses paragraph-level context critical for understanding.

---

## Consequences

### Positive Outcomes

1. **High Retrieval Quality**: Semantic units with overlap improve recall and precision
   - Expect >95% retrieval accuracy on test queries (per success criteria)
   - Cross-boundary questions handled via overlap

2. **Precise Citations**: Chunks map to book paragraphs/sections cleanly
   - Deep links navigate to exact paragraph (paragraph_id)
   - Users can verify information source easily

3. **Better UX**: Chunks are human-readable, complete thoughts
   - Displaying chunk in chat UI shows coherent text
   - Code blocks syntax-highlighted, lists fully visible

4. **Flexible Adaptation**: Variance (100-700 tokens) handles diverse content types
   - Short glossary entries preserved intact
   - Long code blocks not fragmented
   - Tables kept together

### Negative Outcomes & Mitigations

1. **15% Higher Storage & Embedding Cost**:
   - **Impact**: 10k chunks → 11.5k with 15% overlap
   - **Mitigation**: Still within free tier limits (Qdrant: 1M vectors, Cohere: 100 calls/min)
   - **Future**: If costs become issue, reduce overlap to 10% (saves 5%)

2. **Complex Chunking Algorithm**:
   - **Impact**: Semantic boundary detection requires more code than fixed chunking
   - **Mitigation**: Well-tested libraries (LangChain RecursiveCharacterTextSplitter) available
   - **Testing**: Unit tests verify boundary integrity (no mid-sentence splits)

3. **Variable Chunk Sizes**:
   - **Impact**: 100-700 token range complicates batch processing (some chunks outliers)
   - **Mitigation**: Batch by token count, not chunk count (optimize API calls)
   - **Qdrant**: Handles variable payload sizes without issue

4. **Overlap Deduplication Needed**:
   - **Impact**: Top-K retrieval may return overlapping chunks (redundant content)
   - **Mitigation**: Deduplicate by chunk_id in context assembler
   - **UX**: Display unique chunks only, but keep all for context

### Implementation Impact

**Files Affected**:
- `app/services/chunker.py`: Semantic boundary detection, overlap calculation
- `app/models/chunk.py`: `overlap_start`, `overlap_end` fields
- `app/services/embedding_service.py`: Batch processing variable chunk sizes
- `app/services/context_assembler.py`: Deduplicate overlapping chunks in top-K results

**Chunking Algorithm** (pseudocode):
```python
def semantic_chunk(content, target_size=400, min_size=100, max_size=700, overlap=0.15):
    chunks = []
    current_chunk = []
    token_count = 0

    for paragraph in content.split('\n\n'):  # Split on paragraphs
        para_tokens = count_tokens(paragraph)

        if para_tokens > max_size:
            # Split large paragraph at sentence boundaries
            for sentence in split_sentences(paragraph):
                # Add sentence to chunk, check size
                pass
        elif token_count + para_tokens > max_size:
            # Save current chunk, start new chunk with overlap
            overlap_tokens = int(token_count * overlap)
            chunks.append(current_chunk)
            current_chunk = get_last_n_tokens(current_chunk, overlap_tokens)
            token_count = overlap_tokens

        current_chunk.append(paragraph)
        token_count += para_tokens

    return chunks
```

**Metadata Schema**:
- `chunk_id`: UUID
- `text_content`: Full chunk text
- `token_count`: Actual token count (100-700 range)
- `overlap_start`: Number of overlapping tokens from previous chunk (0 if first chunk)
- `overlap_end`: Number of overlapping tokens with next chunk (0 if last chunk)
- `paragraph_id`: Book paragraph identifier for deep linking

**Testing Requirements**:
- **Unit Tests**:
  - Verify no mid-sentence splits
  - Verify code blocks kept intact
  - Verify overlap calculation (10-20% of chunk content)
  - Verify token count variance (100-700 range)
- **Integration Tests**:
  - Chunk sample chapter, verify readability
  - Retrieve with boundary-spanning query, verify both chunks found
- **Performance Tests**:
  - Measure chunking speed (chunks/second)

---

## References

- **Plan Document**: `specs/001-rag-chatbot/plan.md` (Phase 2, Task 2: Semantic Chunker)
- **Specification**: `specs/001-rag-chatbot/spec.md` (FR-002: Chunking strategy, Architectural Decision #10)
- **Clarification Analysis**: `specs/001-rag-chatbot/analysis/clarification-analysis.md` (Q2: Chunking Strategy)
- **Constitution**: `.specify/memory/constitution.md` (Section II: Reliability - 95%+ retrieval accuracy)
- **Related ADRs**:
  - ADR-001 (Cohere Embeddings): Chunk token count affects embedding API costs
  - ADR-002 (Qdrant): Chunk metadata schema (overlap_start, overlap_end)
- **Research**:
  - LangChain Chunking Best Practices: https://python.langchain.com/docs/modules/data_connection/document_transformers/
  - Pinecone RAG Guide (chunk size analysis): https://www.pinecone.io/learn/chunking-strategies/

---

## Acceptance Criteria

This ADR is accepted when:
- [x] Decision clusters chunking strategy (size + overlap + boundaries as integrated solution)
- [x] At least one alternative explicitly listed (4 alternatives documented)
- [x] Clear pros and cons for chosen approach and alternatives
- [x] Consequences cover both positive (quality, UX, flexibility) and negative (cost, complexity) outcomes
- [x] References link back to plan, spec, clarification analysis, and related ADRs
- [x] Implementation impact specified (chunking algorithm, metadata schema, testing)

---

## Review Notes

- **Analyzed**: Decision is architecturally significant (impacts retrieval quality, UX, cost structure across entire RAG pipeline)
- **Measured**: PASS - Clusters chunking strategy (size, overlap, boundaries), lists 4 alternatives, comprehensive tradeoff analysis
- **Risk**: Low - 15% cost increase acceptable, complexity manageable with libraries
- **Dependency**: Affects ADR-002 (Qdrant metadata schema), ADR-001 (embedding costs)
- **Future Work**: If retrieval accuracy <95%, revisit overlap percentage or chunk size bounds
