# ADR-008: Ingestion Timeout Strategy

**Status:** Accepted
**Date:** 2025-12-21
**Deciders:** Development Team
**Related:** ADR-007 (Backend Deployment), Performance optimization

---

## Context

Book ingestion (processing 1713 chunks) takes significantly longer than 3 seconds, causing timeout issues in production:

**Observed Issues:**
- **Local development:** Ingestion completes but takes 2-5 minutes
- **Production deployment:** 504 Gateway Timeout errors
- **Current timeout:** 3 seconds (enforced by request_timeout middleware)
- **Actual duration:** 120-300 seconds depending on Cohere API latency

**Ingestion Pipeline Steps:**
1. Load book files: ~5s
2. Chunk content: ~10s
3. Generate embeddings via Cohere API: ~100-200s (rate-limited)
4. Upsert to Qdrant: ~10-30s
5. Log to database: ~1s

**Total:** 126-246 seconds (well over 3s timeout)

---

## Decision

Implement **background task ingestion** with status polling for large books, while keeping synchronous ingestion available for quick updates.

---

## Rationale

### Why Background Tasks?

**Pros:**
- ✅ **Non-blocking:** Admin can trigger ingestion and check back later
- ✅ **No timeout issues:** No HTTP connection timeout constraints
- ✅ **Better UX:** Immediate feedback that ingestion started
- ✅ **Scalable:** Can handle books of any size
- ✅ **Progress tracking:** Can add progress updates (e.g., "50/1713 chunks processed")
- ✅ **Simple implementation:** FastAPI BackgroundTasks built-in

**Cons:**
- ⚠️ **More complex:** Requires status endpoint and task tracking
- ⚠️ **No immediate result:** Admin must poll for completion
- ⚠️ **Failure discovery delayed:** Errors not immediately visible

---

## Alternatives Considered

### 1. Increase Timeout to 5 Minutes

**Decision:** Rejected as sole solution, but implemented as complement

**Approach:**
```python
# request_timeout.py
if request.url.path != "/api/query":
    return await call_next(request)  # No timeout for ingestion
```

**Pros:**
- Simplest implementation (2-line change)
- Works for synchronous ingestion
- No new code/complexity

**Cons:**
- ❌ **Ties up HTTP connection:** Client waits 5 minutes for response
- ❌ **Poor UX:** No progress feedback during long operation
- ❌ **Proxy timeouts:** Render/Railway may have their own timeouts <5 min
- ❌ **Not scalable:** Fails for larger books (>5 min)

**Verdict:** Use as **complement** for small quick re-indexes, not primary solution

---

### 2. Streaming Response with Server-Sent Events (SSE)

**Decision:** Rejected (over-engineered for admin operation)

**Approach:**
```python
@router.post("/ingest")
async def ingest_book(request: IngestRequest):
    async def event_stream():
        yield f"data: Loading files...\n\n"
        yield f"data: Chunking content...\n\n"
        yield f"data: Generating embeddings (1/100)...\n\n"
        # ...
    return StreamingResponse(event_stream(), media_type="text/event-stream")
```

**Pros:**
- Real-time progress updates
- No polling needed
- Better UX than background tasks

**Cons:**
- ❌ **Complex implementation:** Requires SSE client + server code
- ❌ **Connection management:** Must handle dropped connections
- ❌ **Overkill for admin operation:** Only used once per book update
- ❌ **Not RESTful:** Streaming doesn't fit REST API pattern

**Verdict:** Too complex for infrequent admin operation

---

### 3. Queue-Based Ingestion (Celery/RQ)

**Decision:** Rejected (over-engineered for hackathon scope)

**Approach:**
- Use Celery with Redis backend
- Queue ingestion tasks
- Workers process async

**Pros:**
- Production-grade async task processing
- Built-in retry logic
- Distributed workers possible

**Cons:**
- ❌ **Requires Redis:** Additional service dependency
- ❌ **Complex setup:** Celery configuration, worker processes
- ❌ **Overkill:** Only 1-2 ingestions per month expected
- ❌ **Cost:** Redis adds cost (free tier limits)

**Verdict:** Too complex for hackathon; reconsider for production if >100 ingestions/month

---

### 4. Webhook Callback

**Decision:** Rejected (no external callback URL)

**Approach:**
- Start ingestion
- Return immediately
- Call webhook when complete

**Pros:**
- No polling needed
- Decoupled from client

**Cons:**
- ❌ **No callback URL:** Admin is human, not system
- ❌ **Requires webhook endpoint:** Admin would need to set up receiver
- ❌ **Not user-friendly:** Polling status URL simpler for humans

**Verdict:** Not suitable for human admin workflow

---

## Implementation

### Two-Tier Approach

#### Tier 1: Background Ingestion (Primary)

**Endpoint:** `POST /api/ingest/background`

**Request:**
```json
{
  "book_path": "../textbook/docs",
  "chunk_size": 400,
  "chunk_overlap": 0.15
}
```

**Response (Immediate):**
```json
{
  "task_id": "abc-123-def",
  "status": "started",
  "message": "Ingestion started in background",
  "status_url": "/api/ingest/status/abc-123-def"
}
```

**Status Check:** `GET /api/ingest/status/abc-123-def`

**Response (While Running):**
```json
{
  "task_id": "abc-123-def",
  "status": "running",
  "started_at": "2025-12-21T12:00:00Z",
  "chunks_processed": 850,
  "total_chunks": 1713
}
```

**Response (Completed):**
```json
{
  "task_id": "abc-123-def",
  "status": "completed",
  "started_at": "2025-12-21T12:00:00Z",
  "completed_at": "2025-12-21T12:03:45Z",
  "chunks_processed": 1713,
  "total_chunks": 1713
}
```

#### Tier 2: Synchronous Ingestion (Fallback)

**Endpoint:** `POST /api/ingest` (existing)

**Use Cases:**
- Small updates (<100 chunks)
- Quick re-indexing of single chapter
- Testing

**Timeout:** Excluded from 3s timeout middleware

**Trade-off:** Convenience vs. risk of proxy timeout

---

### Task Storage

**Hackathon/MVP:**
- In-memory dictionary: `_ingestion_tasks: Dict[str, dict]`
- Sufficient for single-server deployment
- Lost on server restart (acceptable)

**Production Migration:**
- Use Redis for task storage
- Persistent across restarts
- Distributed across workers

---

## Consequences

### Positive

1. **Ingestion never times out:** No HTTP connection timeout constraints
2. **Admin gets immediate feedback:** "Started" response in <1s
3. **Can track progress:** Status endpoint shows chunks processed
4. **Scalable:** Works for books of any size
5. **No new dependencies:** Uses FastAPI BackgroundTasks (built-in)

### Negative

1. **Admin must poll:** Check status URL periodically
2. **Task storage in memory:** Lost on restart (non-critical for infrequent operation)
3. **No automatic retry:** If ingestion fails, admin must restart manually
4. **Debugging harder:** Errors not immediately visible to admin

### Mitigations

| Issue | Mitigation |
|-------|-----------|
| Lost tasks on restart | Document in admin guide: "Check status before restarting server" |
| Polling inconvenience | Provide status check script: `./check-ingestion-status.sh <task-id>` |
| No retry logic | Log errors to database for debugging |
| Failure discovery delayed | Email notification on failure (future enhancement) |

---

## Usage Examples

### Admin Workflow (Background Ingestion)

```bash
# 1. Start ingestion
curl -X POST https://backend.com/api/ingest/background \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: secret" \
  -d '{"book_path": "../textbook/docs"}' \
  | jq -r '.task_id' > task_id.txt

# 2. Check status (poll every 30s)
watch -n 30 "curl https://backend.com/api/ingest/status/$(cat task_id.txt)"

# 3. Wait for status: "completed"
# Done!
```

### Admin Workflow (Synchronous Ingestion - Quick Updates)

```bash
# For small updates only (<100 chunks)
curl -X POST https://backend.com/api/ingest \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: secret" \
  -d '{"book_path": "../textbook/docs/chapter1"}'

# Waits for completion (may timeout if >5 min)
```

---

## Future Enhancements

### Short-term (1-3 months)

1. **Progress tracking:**
   ```json
   {
     "status": "running",
     "progress": {
       "current_step": "generating_embeddings",
       "chunks_embedded": 850,
       "total_chunks": 1713,
       "percent_complete": 49.6
     }
   }
   ```

2. **Email notification:**
   - Send email when ingestion completes
   - Include success/failure status
   - Link to logs

3. **Retry logic:**
   - Automatic retry on transient failures
   - Exponential backoff for Cohere API errors

### Long-term (3-6 months)

1. **Celery migration:**
   - Move to Celery for production
   - Redis for task queue
   - Distributed workers

2. **Incremental ingestion:**
   - Detect changed files only
   - Re-index only modified chapters
   - Faster updates (seconds vs minutes)

3. **Webhook support:**
   - Optional webhook URL in request
   - Call webhook on completion
   - Integrate with automation tools

---

## Metrics & Monitoring

### Key Metrics

- **Ingestion duration:** Track min/avg/max time
- **Success rate:** % of ingestions that complete
- **Failure rate:** % that fail, categorize by error type
- **Chunks per second:** Throughput metric

### Alerting

- Alert if ingestion takes >10 minutes (indicates rate limit issue)
- Alert if failure rate >10% (indicates systemic problem)
- Alert if task storage >100 tasks (memory leak indicator)

---

## References

- FastAPI Background Tasks: https://fastapi.tiangolo.com/tutorial/background-tasks/
- Celery Documentation: https://docs.celeryq.dev/
- Server-Sent Events: https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events

---

**Decision Owner:** Development Team
**Review Date:** 2026-01-21 (review after 1 month of production use)
**Status:** ✅ Accepted and Implemented
