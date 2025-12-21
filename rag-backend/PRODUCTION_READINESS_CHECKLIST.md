# Production Readiness Checklist

**Last Updated:** 2025-12-21
**Purpose:** Pre-deployment verification checklist for RAG Chatbot production launch
**See Also:** `DEPLOYMENT.md` for detailed instructions

---

## Pre-Deployment Verification

Use this checklist to verify all production requirements before going live. Check each item and note any blockers.

---

## 1. Backend Deployment ☐

### Platform Setup
- [ ] **Hosting platform selected:** Render.com or Railway.app
- [ ] **Account created** and GitHub connected
- [ ] **Repository connected** to hosting platform
- [ ] **Deploy region selected:** Oregon (US West) or closest to Qdrant cluster

### Build Configuration
- [ ] **Build command set:** `pip install -r requirements.txt`
- [ ] **Start command set:** `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
- [ ] **Root directory set:** `rag-backend` (if monorepo)
- [ ] **Python version specified:** Python 3.10+

### Environment Variables (Backend)
All required environment variables configured in hosting dashboard:

**External Services:**
- [ ] `COHERE_API_KEY` - Valid Cohere API key with trial/paid quota
- [ ] `QDRANT_URL` - Qdrant Cloud cluster URL (https://your-cluster.qdrant.io)
- [ ] `QDRANT_API_KEY` - Qdrant API key with read/write access
- [ ] `DATABASE_URL` - Neon Postgres connection string with `?sslmode=require`

**Security & Access:**
- [ ] `ADMIN_API_KEY` - Secure random key (32+ chars, NOT in git)
- [ ] `ALLOWED_ORIGINS` - Includes Vercel URL: `https://physical-ai-textbook-jet.vercel.app,http://localhost:3000`

**Performance Tuning:**
- [ ] `REQUEST_TIMEOUT_SECONDS` - Set to `300` (5 minutes)
- [ ] `SIMILARITY_THRESHOLD` - Set to `0.7` (or tuned value)

**Optional (has defaults):**
- [ ] `CHUNK_SIZE` - Default: 400
- [ ] `CHUNK_OVERLAP` - Default: 0.15
- [ ] `SEMANTIC_WEIGHT` - Default: 0.7
- [ ] `KEYWORD_WEIGHT` - Default: 0.3

### Deployment Execution
- [ ] **Initial deployment triggered** (automatic or manual)
- [ ] **Build logs verified:** No errors, dependencies installed
- [ ] **Service status:** Running (not failed/crashed)
- [ ] **Public URL obtained:** Note URL for Vercel configuration

---

## 2. Backend Verification ☐

### Health Checks
- [ ] **Health endpoint accessible:**
  ```bash
  curl https://your-backend-url.com/api/health
  # Expected: {"status":"healthy","timestamp":"...","message":"Service is healthy"}
  ```
- [ ] **No 5xx errors** in response
- [ ] **Response time <1s** for health check

### API Documentation
- [ ] **Swagger UI accessible:** `https://your-backend-url.com/api/docs`
- [ ] **All endpoints listed:** /api/query, /api/ingest, /api/ingest/background, /api/ingest/status
- [ ] **API schemas render correctly**

### External Service Connectivity
- [ ] **Qdrant accessible from backend:**
  ```bash
  # Test from backend logs or via query
  curl -X POST https://your-backend-url.com/api/query \
    -H "Content-Type: application/json" \
    -d '{"query_text":"test","selected_text":null}'
  # Should NOT return "INTERNAL_ERROR" if Qdrant connected
  ```
- [ ] **Cohere API accessible** (verify in backend logs during query)
- [ ] **Database accessible** (verify connection in backend startup logs)

### CORS Configuration
- [ ] **CORS headers present** in API responses:
  ```bash
  curl -I https://your-backend-url.com/api/health
  # Should include: Access-Control-Allow-Origin
  ```
- [ ] **Vercel origin allowed** (test from browser console on Vercel site)

---

## 3. Frontend Configuration ☐

### Vercel Environment Variables
- [ ] **Vercel project exists:** `physical-ai-textbook-jet`
- [ ] **Environment variable added:**
  - **Name:** `REACT_APP_API_URL`
  - **Value:** `https://your-backend-url.com` (no trailing slash)
  - **Environments:** Production, Preview, Development (all selected)
- [ ] **Build logs show env var:** Check deployment logs for `REACT_APP_API_URL=...`

### Frontend Deployment
- [ ] **Frontend redeployed** after env var added
- [ ] **Build successful:** No TypeScript/React errors
- [ ] **Deployment live:** `https://physical-ai-textbook-jet.vercel.app` accessible

### Static Asset Verification
- [ ] **Homepage loads** without errors
- [ ] **Chatbot component visible** on homepage
- [ ] **No console errors** in browser DevTools (F12 → Console)
- [ ] **No 404s** for static assets (CSS, JS, images)

---

## 4. End-to-End Integration Testing ☐

### Basic Query Test
- [ ] **Navigate to:** `https://physical-ai-textbook-jet.vercel.app`
- [ ] **Enter test query:** "What is forward kinematics?"
- [ ] **Click "Ask" button**
- [ ] **Verify results:**
  - [ ] Loading state shows ("Thinking...")
  - [ ] Answer appears within 3 seconds
  - [ ] Answer text is relevant and coherent
  - [ ] Citations display below answer
  - [ ] Deep links are clickable
  - [ ] Metadata shows (confidence %, chunks, time)
  - [ ] No errors in browser console

### Error Handling Test
- [ ] **Empty query test:**
  - Enter: "" (empty)
  - Expected: "Please enter a question." error
- [ ] **Non-English test:**
  - Enter: "¿Qué es robótica?" (Spanish)
  - Expected: "Please enter your question in English only." error
- [ ] **Too long test:**
  - Enter: 501+ character query
  - Expected: "Question is too long..." error
- [ ] **Out-of-scope test:**
  - Enter: "What is quantum computing?" (not in textbook)
  - Expected: "I cannot find sufficient information in the textbook..." response

### Citation Deep Link Test
- [ ] **Click a citation link**
- [ ] **Verify:** Opens textbook section in new tab
- [ ] **Verify:** Navigates to correct section (if deep link includes #anchor)
- [ ] **Verify:** No 404 errors

### Performance Test
Open browser DevTools (F12) → Network tab:
- [ ] **Submit query**
- [ ] **Check timing:**
  - [ ] Total time <3s (server enforced timeout)
  - [ ] Typical time <1s (based on Phase 4 testing)
- [ ] **No timeouts or 5xx errors**

---

## 5. Book Content Ingestion ☐

### Pre-Ingestion Verification
- [ ] **Book content exists:** `../textbook/docs` directory contains all chapters
- [ ] **Backend accessible** from local machine (for admin ingestion)
- [ ] **ADMIN_API_KEY available** (stored securely, not in git)
- [ ] **Qdrant collection initialized** (verify in Qdrant dashboard)

### Background Ingestion (Recommended)
- [ ] **Start ingestion:**
  ```bash
  curl -X POST https://your-backend-url.com/api/ingest/background \
    -H "Content-Type: application/json" \
    -H "X-Admin-API-Key: <your-admin-key>" \
    -d '{"book_path":"../textbook/docs","chunk_size":400,"chunk_overlap":0.15}' \
    | jq -r '.task_id'
  ```
- [ ] **Task ID received** (e.g., "abc-123-def")
- [ ] **Status endpoint accessible:**
  ```bash
  curl https://your-backend-url.com/api/ingest/status/<task-id>
  ```
- [ ] **Poll until complete:** Status changes from "running" to "completed"
- [ ] **Verify chunk count:** 1713+ chunks processed
- [ ] **No errors in backend logs**

### Post-Ingestion Verification
- [ ] **Qdrant dashboard:** Vector count shows 1713+ vectors
- [ ] **Database logs:** Ingestion record exists in `ingestion_logs` table
- [ ] **Sample query test:** Query returns relevant results (not "insufficient information")

---

## 6. Monitoring & Observability ☐

### Backend Monitoring Setup
- [ ] **Render/Railway dashboard access** verified
- [ ] **Logs accessible:** Can view real-time logs
- [ ] **Metrics visible:** CPU, RAM, request counts
- [ ] **Health check pinger configured** (optional, to prevent spin-down):
  - Service: UptimeRobot, Cronitor, or custom cron job
  - Interval: Every 5 minutes
  - Endpoint: `https://your-backend-url.com/api/health`

### Frontend Monitoring Setup
- [ ] **Vercel Analytics enabled**
- [ ] **Function logs accessible**
- [ ] **Error tracking configured** (optional: Sentry, LogRocket)

### Alert Configuration (Optional)
- [ ] **Backend downtime alert:** Email/Slack if health check fails >5 min
- [ ] **High error rate alert:** Email/Slack if 5xx errors >10% of requests
- [ ] **Performance degradation alert:** Email/Slack if p95 latency >5s

---

## 7. Security Verification ☐

### Secrets Management
- [ ] **No secrets in git:** `.env` files in `.gitignore`
- [ ] **No API keys in code:** All keys in platform environment variables
- [ ] **ADMIN_API_KEY strong:** 32+ characters, random, not shared publicly

### Network Security
- [ ] **HTTPS enforced:** All URLs use https:// (Render/Vercel automatic)
- [ ] **CORS restricted:** `ALLOWED_ORIGINS` only includes trusted origins
- [ ] **Database SSL enabled:** Connection string includes `?sslmode=require`

### Access Control
- [ ] **Ingestion endpoint protected:** Requires `X-Admin-API-Key` header
- [ ] **Query endpoint public:** No auth required (by design for demo)
- [ ] **Admin key not exposed:** Not in frontend, logs, or public docs

### Rate Limiting
- [ ] **Cohere rate limit configured:** 100 calls/min (enforced in code)
- [ ] **Frontend rate limiting** (optional): Consider adding if abuse risk

---

## 8. Documentation ☐

### User Documentation
- [ ] **README updated** with production URLs
- [ ] **Setup instructions** for local development (PHASE5_SETUP.md)
- [ ] **Deployment guide** complete (DEPLOYMENT.md)

### Technical Documentation
- [ ] **ADRs created:**
  - [ ] ADR-007: Backend Deployment Platform
  - [ ] ADR-008: Ingestion Timeout Strategy
  - [ ] ADR-009: Frontend Environment Variable Strategy
- [ ] **API documentation** accessible (Swagger UI at /api/docs)
- [ ] **Architecture diagram** in DEPLOYMENT.md

### Runbooks
- [ ] **Troubleshooting guide** in DEPLOYMENT.md
- [ ] **Rollback procedure** documented
- [ ] **Ingestion instructions** for admins
- [ ] **Monitoring guide** for ops team

---

## 9. Performance Validation ☐

### Response Time Benchmarks
Based on Phase 4 load testing, verify:
- [ ] **Average query time:** <500ms (target: 300ms)
- [ ] **Max query time:** <3s (enforced by timeout)
- [ ] **Health check time:** <100ms
- [ ] **No cold start delays** (Render: service stays warm)

### Concurrency Testing
- [ ] **10+ concurrent users supported** (Phase 4 validated)
- [ ] **No degradation** under moderate load
- [ ] **Graceful handling** of timeout/overload

### Resource Usage
- [ ] **Backend RAM usage:** <400MB at steady state (free tier: 512MB)
- [ ] **Database connections:** <10 concurrent (free tier limit)
- [ ] **Qdrant query latency:** <500ms average

---

## 10. Rollback & Contingency ☐

### Rollback Readiness
- [ ] **Previous working deployment** identified (in Render/Vercel dashboard)
- [ ] **Rollback procedure tested** (or documented in DEPLOYMENT.md)
- [ ] **Rollback decision criteria** defined (e.g., error rate >25%)

### Contingency Plans
- [ ] **Alternative backend platform** ready (Railway if Render fails)
- [ ] **Backup ADMIN_API_KEY** stored securely
- [ ] **Database backup** strategy (Neon automatic backups verified)
- [ ] **Qdrant snapshot** available (or collection export process documented)

---

## 11. Stakeholder Sign-Off ☐

### Internal Review
- [ ] **Code review completed** (if team policy)
- [ ] **Security review completed** (basic checklist above)
- [ ] **Performance review completed** (benchmarks above)

### Deployment Approval
- [ ] **Product owner notified** of deployment readiness
- [ ] **Go-live date/time confirmed**
- [ ] **User communication plan** (if public launch announcement)

---

## 12. Post-Deployment Validation ☐

### Immediate Post-Launch (15 min)
- [ ] **Health check passing**
- [ ] **Sample query successful**
- [ ] **No 5xx errors in logs**
- [ ] **Frontend accessible**
- [ ] **No CORS errors**

### Short-Term Monitoring (1 hour)
- [ ] **Response times stable** (<3s)
- [ ] **Error rate <1%**
- [ ] **No memory leaks** (RAM usage stable)
- [ ] **User feedback** (if available)

### Long-Term Validation (24 hours)
- [ ] **No service crashes**
- [ ] **Uptime >99%**
- [ ] **Performance within SLO**
- [ ] **User-reported issues** addressed

---

## Final Production Readiness Status

### Summary Checklist

**Critical Requirements:**
- [ ] Backend deployed and healthy
- [ ] Frontend connected to backend
- [ ] CORS configured correctly
- [ ] All environment variables set
- [ ] Book content ingested (1713+ chunks)
- [ ] End-to-end query test passed
- [ ] Security checklist complete
- [ ] Documentation complete

**Ready for Production:** ☐ Yes ☐ No

**Blocker Issues:**
1. _____________________________________________________
2. _____________________________________________________
3. _____________________________________________________

**Risk Assessment:**
- **High:** _____________________________________________________
- **Medium:** _____________________________________________________
- **Low:** _____________________________________________________

**Go/No-Go Decision:** ☐ GO ☐ NO-GO

**Approved By:** _____________________ **Date:** __________

---

## Quick Reference Commands

### Backend Health Check
```bash
curl https://your-backend-url.com/api/health
```

### Test Query (Backend Direct)
```bash
curl -X POST https://your-backend-url.com/api/query \
  -H "Content-Type: application/json" \
  -d '{"query_text":"What is robotics?","selected_text":null}'
```

### Background Ingestion
```bash
# Start
curl -X POST https://your-backend-url.com/api/ingest/background \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: <key>" \
  -d '{"book_path":"../textbook/docs"}' | jq -r '.task_id'

# Check status
curl https://your-backend-url.com/api/ingest/status/<task-id>
```

### Check Vercel Deployment
```bash
vercel inspect <deployment-url>
```

### View Backend Logs (Railway)
```bash
railway logs --tail
```

### View Backend Logs (Render)
Visit: Dashboard → Logs

---

**Checklist Version:** 1.0
**Last Updated:** 2025-12-21
**Maintained By:** Development Team
**Next Review:** 2026-01-21 (1 month post-launch)
