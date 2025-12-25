# ADR-007: Backend Deployment Platform Choice

**Status:** Accepted
**Date:** 2025-12-21
**Deciders:** Development Team
**Related:** Production deployment integration

---

## Context

The RAG Chatbot backend (FastAPI) needs to be deployed to a cloud platform for production use. The frontend is already deployed on Vercel at `https://physical-ai-textbook-jet.vercel.app`, but the backend is running locally at `http://127.0.0.1:8000`, which is not accessible from Vercel's cloud infrastructure.

**Requirements:**
- Free tier available for hackathon/MVP
- Python/FastAPI support
- Easy GitHub integration
- Environment variable management
- Minimal cold start latency
- SSL/TLS support
- Monitoring and logs

---

## Decision

Use **Render.com** free tier for backend deployment.

---

## Rationale

### Why Render.com?

**Pros:**
- ✅ **Free tier available:** 512MB RAM, shared CPU (sufficient for demo/MVP)
- ✅ **Native Python support:** No need for custom Dockerfiles
- ✅ **Easy GitHub integration:** Automatic deploys on push
- ✅ **No cold starts:** Service stays warm unlike serverless options
- ✅ **Built-in SSL/TLS:** Automatic HTTPS certificates
- ✅ **Environment variables:** Dashboard GUI for secrets management
- ✅ **Monitoring:** Built-in metrics, logs, and health checks
- ✅ **Simple configuration:** `render.yaml` or dashboard setup
- ✅ **Fast deployment:** 5-10 minutes first deploy, <2 min updates

**Cons:**
- ⚠️ **Free tier limitations:**
  - 512MB RAM (may need upgrade for heavy load)
  - Shared CPU (slower than dedicated)
  - Service spins down after 15 min inactivity (restart takes 30s)
- ⚠️ **Region options limited:** Oregon, Frankfurt, Singapore only
- ⚠️ **No built-in database:** Must use external (Neon Postgres)

---

## Alternatives Considered

### 1. Railway.app

**Decision:** Rejected as primary, offered as alternative

**Pros:**
- Simple CLI deployment
- Automatic HTTPS
- Great developer experience
- Built-in database options

**Cons:**
- Free tier more limited (500 hours/month vs unlimited on Render)
- Less mature than Render for production workloads
- Documentation less comprehensive

**Verdict:** Good alternative if Render doesn't work, but Render's free tier is more generous

---

### 2. Vercel Serverless Functions

**Decision:** Rejected

**Pros:**
- Frontend already on Vercel (single platform)
- Automatic scaling
- Zero cost at low traffic

**Cons:**
- ❌ **Cold starts:** 2-5s latency on first request (violates <3s requirement)
- ❌ **10-second timeout:** Ingestion would fail (requires >5 min)
- ❌ **Limited Python support:** Requires custom build configuration
- ❌ **No persistent connections:** Cannot maintain database pool

**Verdict:** Serverless incompatible with long-running ingestion and persistent connections

---

### 3. Fly.io

**Decision:** Rejected

**Pros:**
- Global edge deployment
- Good performance
- Docker-first approach

**Cons:**
- ❌ **More complex configuration:** Requires Dockerfile, flyctl setup
- ❌ **Steeper learning curve:** More manual configuration
- ❌ **Free tier less clear:** Credit-based system

**Verdict:** More complexity than needed for hackathon scope

---

### 4. Heroku

**Decision:** Rejected

**Pros:**
- Mature platform
- Good Python support
- Simple deployment

**Cons:**
- ❌ **No free tier:** Removed in 2022, minimum $5-7/month
- ❌ **Over budget:** Outside hackathon constraints

**Verdict:** Too expensive for free-tier requirement

---

## Implementation

### Deployment Configuration

**Build Command:**
```bash
pip install -r requirements.txt
```

**Start Command:**
```bash
uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

**Environment Variables:**
```bash
COHERE_API_KEY=<secret>
QDRANT_URL=<url>
QDRANT_API_KEY=<secret>
DATABASE_URL=postgresql://...?sslmode=require
ALLOWED_ORIGINS=https://physical-ai-textbook-jet.vercel.app,http://localhost:3000
REQUEST_TIMEOUT_SECONDS=300
SIMILARITY_THRESHOLD=0.7
ADMIN_API_KEY=<secret>
```

**Region Choice:**
- **Oregon (US West):** Default choice
- Rationale: Closest to Qdrant cluster if using US region
- Alternative: Frankfurt if Qdrant cluster in EU

---

## Consequences

### Positive

1. **Fast deployment:** Hackathon-ready in <1 hour setup time
2. **No cost:** Free tier sufficient for demo and initial users
3. **Production-ready:** Can upgrade to paid tier if needed
4. **Monitoring:** Built-in logs and metrics for debugging
5. **HTTPS:** Automatic SSL certificates, no configuration needed

### Negative

1. **Cold starts on free tier:** Service spins down after 15 min inactivity
   - Mitigation: Use health check pinger (UptimeRobot, cron job)
   - Impact: First request after spin-down takes ~30s

2. **RAM limitations:** 512MB may not be enough for heavy load
   - Mitigation: Upgrade to paid tier ($7/month for 2GB RAM)
   - Impact: May hit memory limits with >50 concurrent users

3. **Regional latency:** Limited to 3 regions (Oregon, Frankfurt, Singapore)
   - Mitigation: Choose region closest to Qdrant cluster
   - Impact: Higher latency for users far from region

### Risks & Mitigations

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Service spin-down | High (free tier) | Medium (30s delay) | Health check pinger |
| Out of memory | Low (demo traffic) | Medium (503 errors) | Monitor metrics, upgrade tier |
| Regional latency | Medium | Low (<100ms) | Choose optimal region |
| Render outage | Low | High (full downtime) | Have Railway deployment script ready |

---

## Future Considerations

### When to Upgrade

**Indicators:**
- >50 concurrent users
- Response times >5s consistently
- Out of memory errors in logs
- Need for auto-scaling
- 99.9% SLA requirement

**Upgrade Path:**
1. Render Starter ($7/month): 2GB RAM, faster CPU
2. Render Standard ($25/month): 4GB RAM, dedicated CPU
3. Self-hosted: Migrate to AWS/GCP with Kubernetes

### When to Reconsider Platform

**Switch to Railway if:**
- Need built-in PostgreSQL (Render requires external)
- Prefer CLI-first deployment workflow
- Want more deployment flexibility

**Switch to Cloud Provider (AWS/GCP) if:**
- Need auto-scaling beyond Render's capabilities
- Require multi-region deployment
- Need VPC for enhanced security
- Traffic exceeds Render's paid tiers

---

## References

- Render Documentation: https://render.com/docs
- Render Python Guide: https://render.com/docs/deploy-fastapi
- Render Free Tier: https://render.com/pricing
- Railway Comparison: https://railway.app/pricing
- Fly.io Comparison: https://fly.io/docs/about/pricing

---

**Decision Owner:** Development Team
**Review Date:** 2026-01-21 (1 month after deployment)
**Status:** ✅ Accepted and Implemented
