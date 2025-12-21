---
id: 009
title: Document Indexing Strategy ADR
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.adr
labels: ["adr", "indexing", "re-indexing", "incremental", "manual-trigger", "automation", "architecture"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend/history/adr/006-manual-indexing-for-hackathon.md
  pr: null
files:
  - history/adr/006-manual-indexing-for-hackathon.md
tests:
  - none
---

## Prompt

/sp.adr Manual Indexing vs Automated Rebuilds

## Response snapshot

**ADR-006 Created**: Manual Indexing for Hackathon (Incremental Re-Indexing Future)

**Decision**: Use manual trigger for hackathon scope, with incremental re-indexing designed for future implementation

**Context**:
- Need strategy for handling book content updates and re-indexing
- Requirements: Reflect updates within 24 hours (SC-010), minimize re-indexing cost, free tier constraints, clear operational procedures
- Update frequency: Rare (monthly or less) during hackathon, may increase to weekly/daily in production
- Re-indexing complexity: Full rebuild (10k chunks, 5-10 mins, 10k API calls) vs incremental (detect changes, re-embed only affected chunks)
- Constraints: Hackathon timeline (24-48 hrs), free tier limits (100 calls/min), admin access, 24-hour staleness tolerance

**Rationale** (2 phases):
1. **Hackathon: Manual Trigger**: Admin calls `POST /api/ingest` endpoint when book updated
   - Simplicity: No automation overhead, saves 3-4 hrs development (no webhook, no change detection)
   - Update frequency: Rare updates (monthly or less), low manual effort (10 seconds to call API)
   - Free tier viable: Full rebuild 10k chunks in 5-10 mins (batching: 96 texts/request = 105 API calls)
   - Operational flow: Update Docusaurus → call `/api/ingest` → delete all Qdrant chunks → re-chunk and re-embed entire book → upsert

2. **Future: Incremental Re-Indexing**: Detect changed sections, re-embed only affected chunks
   - Cost optimization: Typical update (3 paragraphs) = 5 chunks = 5 API calls (99%+ reduction vs 10k)
   - Speed: Re-embed 5 chunks in <10 seconds (vs 5-10 minutes for full rebuild)
   - Scalability: Automated webhook on Docusaurus build, no manual intervention
   - Change detection: File hash (SHA256 stored in Postgres), compare new vs stored hash
   - Implementation: Delete old chunks by `file_path` filter, re-chunk changed files only, upsert new chunks

**Tradeoffs**:
- **Pros**: Faster hackathon dev (3-4 hrs saved), simple operations (clear README instructions), acceptable staleness (24-hr tolerance), future-ready design (modular `/api/ingest` can switch to incremental internally)
- **Cons**: Manual effort (admin must remember to trigger, mitigated by README + Docusaurus dev notes), full rebuild cost (10k API calls, acceptable for rare updates, future incremental fixes), staleness risk (if admin forgets, mitigated by banner if Git last commit > last ingest timestamp), no change tracking (full rebuild always, mitigated by storing hashes during ingest)

**Alternatives Considered** (4):
1. **Automated Webhook (Docusaurus Build → Ingest)**: Zero manual effort, immediate updates, but setup complexity (2-3 hrs), wasteful rebuilds (triggers on CSS changes), still full rebuild (10k calls), hackathon overkill (rejected: better post-hackathon with incremental)
2. **Scheduled Nightly Re-Indexing**: Automated, predictable (2 AM), but wasteful (99% no changes), free tier risk (daily full rebuilds), latency (up to 24 hrs if change at 2:01 AM), infrastructure (cron job) (rejected: wasteful for rare updates)
3. **Full Rebuild Every Time (No Incremental)**: Simplest, always consistent, but slow (5-10 mins), expensive (10k API calls), doesn't scale (50k chunks = 50 mins), rate limits (100 calls/min = 100 mins without batching) (rejected for future, acceptable for hackathon)
4. **Real-Time Incremental (On Every Save)**: Instant updates, maximum responsiveness, but complexity (file watcher, debouncing), 6-8 hrs development, unnecessary (24-hr tolerance sufficient) (rejected: overkill)

**Consequences**:
- **Positive**: Faster development (3-4 hrs saved), simple operations (README instructions), acceptable staleness (24-hr tolerance, manual trigger can wait), future-ready (modular design supports incremental drop-in)
- **Negative**: Manual effort (admin must trigger, mitigated by docs), full rebuild cost (10k API calls, batching reduces to 105 calls), staleness risk (admin forgets, mitigated by banner), no change tracking (store hashes for future)

**Implementation Impact**:
- **Hackathon**:
  - Files: `ingest.py` (`POST /api/ingest` endpoint, admin only), `ingestion_service.py` (full rebuild logic), `README.md` (operational instructions)
  - Flow: Clear Qdrant (`DELETE /collections/textbook_chunks/points`) → load book files → chunk and embed → bulk upsert
  - Code: `qdrant.delete_collection()` → `load_book_files()` → `semantic_chunker.chunk()` → `cohere.embed()` → `qdrant.upsert()`
- **Future Incremental**:
  - Files: `book_content.py` (`BookContentHash` model), `change_detector.py` (compute SHA256, compare hashes), `ingestion_service.py` (incremental logic)
  - Flow: Load files → detect changes (hash comparison) → delete old chunks by `file_path` filter → re-chunk changed files → embed → upsert new chunks
  - Code: `detect_changes()` (SHA256 hash, compare with stored) → `qdrant.delete(filter=file_path)` → re-chunk/embed changed files only → `qdrant.upsert()`
- Testing: Unit (hackathon: `/api/ingest` auth, full rebuild; future: change detection, incremental delete/upsert), integration (hackathon: ingest sample, update, verify; future: update single chapter, verify ~50 chunks not 10k, <1 min)

**Related ADRs**:
- ADR-001 (Cohere Embeddings): Re-indexing consumes embedding API calls
- ADR-002 (Qdrant): Delete and upsert operations for re-indexing
- ADR-003 (Chunking): Re-chunking algorithm applied during re-indexing

## Outcome

- ✅ Impact: Formalized indexing strategy; defines ingestion pipeline operations, admin procedures, future scalability path
- 🧪 Tests: Defined 2 test categories per phase (unit: endpoint auth + rebuild logic, integration: full workflow verification)
- 📁 Files: Created history/adr/006-manual-indexing-for-hackathon.md (5.6KB)
- 🔁 Next prompts: All 6 ADRs complete, proceed to /sp.tasks
- 🧠 Reflection: ADR clusters indexing strategy (hackathon manual trigger + future incremental + change detection as integrated solution), with 4 alternatives and comprehensive tradeoff analysis. Manual trigger justified by rare updates and hackathon timeline, with clear migration path to automated incremental re-indexing post-hackathon via file hash change detection.

## Evaluation notes (flywheel)

- Failure modes observed: None; decision well-documented in plan.md and spec.md (FR-016, FR-019, SC-010, Assumption #4, Best Practice: incremental)
- Graders run and results (PASS/FAIL):
  - Analyze: PASS - Clusters indexing strategy (not over-granular), lists 4 alternatives with detailed rationale
  - Measure: PASS - Architecturally significant (impacts ingestion pipeline, operational procedures, API design, future scalability), explicit alternatives, clear pros/cons, comprehensive detail with both hackathon and future implementation code
- Prompt variant (if applicable): N/A - standard /sp.adr workflow
- Next experiment (smallest change to try): All 6 ADRs completed; validate consistency across all ADRs, then proceed to /sp.tasks to generate implementation task breakdown
