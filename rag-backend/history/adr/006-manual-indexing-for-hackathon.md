# ADR-006: Manual Indexing for Hackathon (Incremental Re-Indexing Future)

**Status**: Accepted
**Date**: 2025-12-20
**Deciders**: Architecture Team
**Feature**: 001-rag-chatbot

---

## Context

The RAG chatbot requires a strategy for handling book content updates and re-indexing. When the textbook content changes (chapters edited, glossary updated, new sections added), the embedding vectors and chunks in Qdrant must be updated to reflect the new content.

**Key Requirements**:
- Support book content updates (chapters, glossary, references, code examples)
- Reflect updated content in search results within 24 hours (per SC-010)
- Minimize re-indexing cost (embedding API calls, Qdrant operations)
- Work within free tier constraints (Cohere: 100 calls/min, Qdrant: 1M vectors)
- Provide clear operational procedures for hackathon demo and early usage

**Update Frequency Assumptions**:
- **Hackathon period**: Book updates rare (monthly or less)
- **Production**: May increase to weekly or daily as textbook evolves
- **Update types**:
  - Minor edits (typos, clarifications) - affect 1-5 chunks
  - Major updates (new sections, rewrites) - affect 10-100 chunks
  - New chapters - affect 500-1000 chunks

**Re-Indexing Complexity**:
- **Full rebuild**: Re-embed all ~10k chunks (takes 5-10 minutes, costs 10k API calls)
- **Incremental**: Detect changed sections, re-embed only affected chunks (faster, cheaper)
- **Detection**: Need mechanism to identify what changed (file hash, timestamp, content diff)

**Constraints**:
- Hackathon timeline: 24-48 hours total development time
- Free tier API limits: 100 Cohere calls/min (full rebuild = 100 minutes)
- Admin access: Developer can manually trigger re-indexing if needed
- Staleness tolerance: 24 hours acceptable per spec (SC-010)

---

## Decision

**We will use manual trigger for hackathon scope, with incremental re-indexing designed for future implementation.**

This decision includes:
- **Hackathon Approach**: Manual trigger via admin API endpoint (`POST /api/ingest`)
  - Admin (developer) calls endpoint when book content updated
  - Full rebuild of all chunks (simple, no change detection logic needed)
  - Documented in README with clear operational instructions
- **Future Approach**: Incremental re-indexing with change detection
  - Detect changed sections via file hash or last_modified timestamp
  - Delete old chunks for changed sections, insert new chunks
  - Only re-embed affected chunks (faster, cheaper)
  - Modular design allows drop-in replacement of manual trigger

**Operational Flow (Hackathon)**:
1. Developer updates textbook content in Docusaurus
2. Developer manually calls `POST /api/ingest` endpoint
3. Backend deletes all existing chunks from Qdrant
4. Backend re-chunks and re-embeds entire book (~10k chunks, 5-10 minutes)
5. Users see updated content in next query

**Future Extension (Post-Hackathon)**:
1. Developer updates textbook, commits to Git
2. Docusaurus build webhook triggers `/api/ingest` automatically
3. Backend computes file hashes, compares with stored hashes in Postgres
4. Backend identifies changed files, extracts affected chunks
5. Backend deletes old chunks, re-embeds only new/changed chunks
6. Users see updated content within minutes (not 5-10 minutes)

---

## Rationale

### Why Manual Trigger for Hackathon

**Simplicity**:
- **No automation overhead**: No webhook setup, no change detection logic
  - Saves 3-4 hours of development time
  - Reduces testing surface (no webhook validation, no hash comparison)
- **Straightforward implementation**: Single API endpoint, full rebuild
- **Sufficient for demo**: Book updates rare during hackathon, manual trigger acceptable

**Update Frequency**:
- **Rare updates**: Book content stable during hackathon (monthly or less per assumption)
- **Low manual effort**: Calling API endpoint takes 10 seconds (vs 4 hours to automate)
- **Acceptable staleness**: 24-hour tolerance (SC-010) means manual trigger can wait until convenient

**Free Tier Capacity**:
- **Full rebuild viable**: 10k chunks × 1 call = 10k API calls, within daily limits
- **Time acceptable**: 5-10 minutes for full rebuild (100 calls/min limit = 100 minutes worst case, batching reduces to ~10 mins)

### Why Incremental Re-Indexing for Future

**Cost Optimization**:
- **Typical update**: Edit 3 paragraphs → affects 5 chunks → 5 API calls (vs 10k for full rebuild)
- **Major update**: New section → affects 50 chunks → 50 API calls (vs 10k)
- **Savings**: 99%+ reduction in API calls for minor updates

**Speed**:
- **Incremental**: Re-embed 5 chunks in <10 seconds (vs 5-10 minutes for full rebuild)
- **User impact**: Content available within minutes, not 10 minutes

**Scalability**:
- **Production**: If book updates become daily, manual trigger doesn't scale (admin fatigue)
- **Automated**: Webhook triggers on Docusaurus build, no manual intervention

**Change Detection Strategy**:
- **File hash**: SHA256 of each file, stored in Postgres `book_content_hashes` table
  - Compare new hash with stored hash, detect changes
- **Timestamp**: `last_modified` from filesystem or Git commit timestamp
  - Less reliable (filesystem timestamps can be incorrect)
- **Content diff**: Compare text directly, detect changed paragraphs
  - Most accurate, but highest complexity

---

## Alternatives Considered

### Alternative 1: Automated Webhook Trigger (Docusaurus Build → Ingest)

**Approach**: Configure Docusaurus build hook to call `/api/ingest` automatically on each build

**Pros**:
- Zero manual effort (no admin intervention needed)
- Immediate updates (content reflected within 5-10 minutes of build)
- Scalable for production (handles daily updates)

**Cons**:
- ❌ **Setup complexity**: Configure webhook in Docusaurus, secure endpoint (API key)
- ❌ **Development time**: 2-3 hours to implement and test webhook
- ❌ **Wasteful rebuilds**: Triggers on every build (even non-content changes like CSS)
- ❌ **No incremental logic**: Still does full rebuild (10k API calls each time)
- ❌ **Hackathon overkill**: Book updates rare, automation not justified

**Verdict**: Rejected for hackathon. Consider post-hackathon with incremental logic to avoid wasteful rebuilds.

---

### Alternative 2: Scheduled Nightly Re-Indexing

**Approach**: Cron job runs nightly, checks for changes, re-indexes if needed

**Pros**:
- Automated (no manual trigger)
- Predictable (always runs at 2 AM, low user traffic)
- Detects changes via file hash comparison

**Cons**:
- ❌ **Wasteful**: Runs every night even if no changes (99% wasted checks)
- ❌ **Free tier risk**: Daily full rebuilds consume API quota unnecessarily
- ❌ **Latency**: Up to 24 hours before updates reflected (if change happens at 2:01 AM)
- ❌ **Infrastructure**: Requires cron job setup (server or cloud scheduler)

**Verdict**: Rejected. Wasteful for rare updates, doesn't meet 24-hour requirement if change happens right after scheduled run.

---

### Alternative 3: Full Rebuild Every Time (No Incremental Logic)

**Approach**: Always re-embed entire book, never detect changes or do incremental

**Pros**:
- Simplest implementation (no change detection)
- Always fully consistent (no risk of stale chunks)
- No database tracking needed (no hash storage)

**Cons**:
- ❌ **Slow**: 5-10 minutes per rebuild (bad UX if frequent updates)
- ❌ **Expensive**: 10k API calls per rebuild (exhausts free tier quickly)
- ❌ **Doesn't scale**: If book grows to 50k chunks, takes 50 minutes + 50k API calls
- ❌ **API rate limits**: 100 calls/min means 100 minutes for 10k chunks (without batching)

**Verdict**: Rejected for future, but acceptable for hackathon (rare updates). Post-hackathon must switch to incremental.

---

### Alternative 4: Real-Time Incremental (On Every Save)

**Approach**: Detect changes on every file save, immediately re-index affected chunks

**Pros**:
- Instant updates (no delay)
- Maximum responsiveness
- Fine-grained (only re-embeds exact changed chunks)

**Cons**:
- ❌ **Complexity**: File watcher, debouncing (avoid re-indexing on temp saves), change detection
- ❌ **Development time**: 6-8 hours (file watcher, incremental logic, testing)
- ❌ **Not hackathon viable**: Too much scope for 24-48 hour timeline
- ❌ **Unnecessary**: 24-hour staleness acceptable per spec (SC-010)

**Verdict**: Rejected. Overkill for hackathon and even post-hackathon (24-hour tolerance sufficient).

---

## Consequences

### Positive Outcomes

1. **Faster Hackathon Development**: 3-4 hours saved by skipping automation
   - No webhook setup, no change detection logic, no hash storage
   - Focus on core RAG functionality (retrieval, generation, citations)

2. **Simple Operations**: Clear admin instructions in README
   - "To update book content: edit Docusaurus files, then call `POST /api/ingest`"
   - Low operational complexity, easy to demo

3. **Acceptable Staleness**: 24-hour tolerance (SC-010) means manual trigger can wait
   - Admin can trigger at convenient time (not urgent)
   - Users see banner if content stale: "Textbook updated, re-indexing pending"

4. **Future-Ready Design**: Modular architecture supports incremental drop-in
   - `/api/ingest` endpoint can switch from full rebuild to incremental internally
   - No API contract changes needed

### Negative Outcomes & Mitigations

1. **Manual Effort Required**:
   - **Impact**: Admin must remember to trigger re-indexing after updates
   - **Mitigation**: Document procedure in README, add reminder in Docusaurus dev notes
   - **Future**: Automate via webhook post-hackathon

2. **Full Rebuild Cost**:
   - **Impact**: 10k API calls per rebuild (acceptable for rare updates, not scalable)
   - **Mitigation**: Batching reduces time (96 texts/request = 105 API calls, ~10 minutes)
   - **Future**: Incremental logic reduces to <100 API calls for typical updates

3. **Staleness Risk**:
   - **Impact**: If admin forgets to trigger, users see stale content (violates SC-010)
   - **Mitigation**: Display banner if book content updated (check Git last commit vs last ingest timestamp)
   - **Future**: Automated webhook eliminates human error

4. **No Change Tracking**:
   - **Impact**: Cannot identify which chunks changed (full rebuild always)
   - **Mitigation**: Store file hashes in Postgres during ingest (prep for future incremental)
   - **Future**: Use stored hashes to detect changed files

### Implementation Impact

**Files Affected (Hackathon)**:
- `app/api/routes/ingest.py`: `POST /api/ingest` endpoint (admin only, clears Qdrant, re-chunks, re-embeds)
- `app/services/ingestion_service.py`: Full rebuild logic
- `rag-backend/README.md`: Operational instructions for manual trigger
- Qdrant: `DELETE /collections/textbook_chunks/points` (clear all), then bulk upsert

**Files Affected (Future Incremental)**:
- `app/models/book_content.py`: `BookContentHash` model (file_path, hash, last_updated)
- `app/services/change_detector.py`: Compute file hashes, compare with stored hashes
- `app/services/ingestion_service.py`: Incremental logic (delete old chunks, insert new chunks)
- Qdrant: Targeted delete by `file_path` filter, selective upsert

**Hackathon Implementation**:
```python
# app/api/routes/ingest.py
@router.post("/ingest")
async def ingest_book_content(api_key: str = Header(None)):
    """
    Admin-only endpoint to re-index entire book.
    Manual trigger for hackathon scope.
    """
    if api_key != settings.ADMIN_API_KEY:
        raise HTTPException(403, "Unauthorized")

    # Full rebuild: clear Qdrant, re-chunk, re-embed
    await qdrant.delete_collection("textbook_chunks")
    await qdrant.create_collection("textbook_chunks", vector_size=1024, distance="Cosine")

    # Fetch book content from Docusaurus (file system or API)
    book_files = load_book_files("/path/to/docusaurus/docs")

    # Chunk and embed
    all_chunks = []
    for file in book_files:
        chunks = semantic_chunker.chunk(file.content)
        embeddings = await cohere.embed(chunks)
        all_chunks.extend(zip(chunks, embeddings))

    # Upsert to Qdrant
    await qdrant.upsert(collection_name="textbook_chunks", points=all_chunks)

    return {"status": "success", "chunks_indexed": len(all_chunks)}
```

**Future Incremental Implementation**:
```python
# app/services/change_detector.py
def detect_changes(book_files: List[BookFile]) -> List[BookFile]:
    """
    Compute file hashes, compare with stored hashes, return changed files.
    """
    changed_files = []
    for file in book_files:
        new_hash = hashlib.sha256(file.content.encode()).hexdigest()
        stored_hash = db.query(BookContentHash).filter_by(file_path=file.path).first()

        if not stored_hash or stored_hash.hash != new_hash:
            changed_files.append(file)
            # Update hash in database
            db.merge(BookContentHash(file_path=file.path, hash=new_hash, last_updated=datetime.now()))

    db.commit()
    return changed_files

# app/api/routes/ingest.py (modified for incremental)
@router.post("/ingest")
async def ingest_book_content(api_key: str = Header(None)):
    book_files = load_book_files("/path/to/docusaurus/docs")
    changed_files = change_detector.detect_changes(book_files)

    if not changed_files:
        return {"status": "no_changes", "chunks_indexed": 0}

    # Delete old chunks for changed files
    for file in changed_files:
        await qdrant.delete(
            collection_name="textbook_chunks",
            points_selector={"filter": {"must": [{"key": "file_path", "match": {"value": file.path}}]}}
        )

    # Re-chunk and re-embed only changed files
    new_chunks = []
    for file in changed_files:
        chunks = semantic_chunker.chunk(file.content)
        embeddings = await cohere.embed(chunks)
        new_chunks.extend(zip(chunks, embeddings))

    # Upsert new chunks
    await qdrant.upsert(collection_name="textbook_chunks", points=new_chunks)

    return {"status": "success", "files_changed": len(changed_files), "chunks_indexed": len(new_chunks)}
```

**Testing Requirements**:
- **Unit Tests (Hackathon)**:
  - Test `/api/ingest` endpoint (unauthorized returns 403, authorized triggers rebuild)
  - Test full rebuild logic (clear Qdrant, re-chunk, re-embed, upsert)
- **Integration Tests (Hackathon)**:
  - Ingest sample book, query chatbot, verify results
  - Update book content, call `/api/ingest`, verify updated results
- **Unit Tests (Future Incremental)**:
  - Test change detection (new hash vs stored hash)
  - Test incremental delete (only affected chunks deleted)
  - Test incremental upsert (only new chunks added)
- **Integration Tests (Future Incremental)**:
  - Update single chapter, verify only ~50 chunks re-indexed (not 10k)
  - Measure time (should be <1 minute for single chapter update)

---

## References

- **Plan Document**: `specs/001-rag-chatbot/plan.md` (ADR-006: Online Indexing vs Offline Rebuilds)
- **Specification**: `specs/001-rag-chatbot/spec.md` (FR-016: Ingestion API endpoint, FR-019: Detect updates and trigger re-ingestion, SC-010: Reflect updates within 24 hours, Assumption #4: Infrequent updates, Best Practice: Prefer incremental re-indexing)
- **Constitution**: `.specify/memory/constitution.md` (Section V: Maintainability - modular design for future extensibility)
- **Related ADRs**:
  - ADR-001 (Cohere Embeddings): Re-indexing consumes embedding API calls
  - ADR-002 (Qdrant): Delete and upsert operations for re-indexing
  - ADR-003 (Chunking): Re-chunking algorithm applied during re-indexing
- **Implementation Reference**:
  - Qdrant Delete API: https://qdrant.tech/documentation/concepts/points/#delete-points
  - SHA256 File Hashing: Python `hashlib.sha256()`

---

## Acceptance Criteria

This ADR is accepted when:
- [x] Decision clusters indexing strategy (hackathon manual + future incremental as integrated solution)
- [x] At least one alternative explicitly listed (4 alternatives documented)
- [x] Clear pros and cons for chosen approach and alternatives
- [x] Consequences cover both positive (speed, simplicity, future-ready) and negative (manual effort, full rebuild cost, staleness risk) outcomes
- [x] References link back to plan, spec, constitution, and related ADRs
- [x] Implementation impact specified (manual trigger endpoint, future incremental logic, change detection)

---

## Review Notes

- **Analyzed**: Decision is architecturally significant (impacts ingestion pipeline, operational procedures, API design, future scalability)
- **Measured**: PASS - Clusters indexing strategy (manual trigger + incremental future + change detection), lists 4 alternatives, comprehensive tradeoff analysis with code
- **Risk**: Low - Manual trigger acceptable for hackathon (rare updates), clear migration path to incremental post-hackathon
- **Dependency**: Affects ADR-001 (embedding API cost), ADR-002 (Qdrant delete/upsert), ADR-003 (re-chunking)
- **Future Work**: Implement incremental re-indexing post-hackathon when book updates become more frequent (weekly or daily)
