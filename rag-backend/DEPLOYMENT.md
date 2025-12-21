# Production Deployment Guide

## Overview

This guide provides step-by-step instructions for deploying the RAG Chatbot system to production, integrating the Vercel-hosted frontend with a cloud-deployed backend.

**System Architecture:**
```
Vercel Frontend (https://physical-ai-textbook-jet.vercel.app)
         ↓ HTTPS
Cloud Backend (Render/Railway)
         ↓
    ┌────────────┬──────────┬─────────────┐
    │ Cohere API │ Qdrant   │ Neon Postgres│
    └────────────┴──────────┴─────────────┘
```

---

## Prerequisites

### Required Services

1. **Vercel Account** (frontend hosting)
   - Sign up: https://vercel.com/signup
   - CLI: `npm install -g vercel`

2. **Backend Hosting** (choose one):
   - **Render.com** (Recommended): https://render.com
   - **Railway.app** (Alternative): https://railway.app

3. **External Services** (already configured):
   - Cohere API key
   - Qdrant Cloud cluster
   - Neon Serverless Postgres database

### Required Environment Variables

Create these in your backend hosting platform:

```bash
# Required
COHERE_API_KEY=<your-cohere-api-key>
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=<your-qdrant-api-key>
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
ADMIN_API_KEY=<generate-secure-random-key-here>

# Required for CORS
ALLOWED_ORIGINS=https://physical-ai-textbook-jet.vercel.app,http://localhost:3000

# Performance tuning
REQUEST_TIMEOUT_SECONDS=300  # 5 min for ingestion
SIMILARITY_THRESHOLD=0.7

# Optional (has defaults)
CHUNK_SIZE=400
CHUNK_OVERLAP=0.15
SEMANTIC_WEIGHT=0.7
KEYWORD_WEIGHT=0.3
```

---

## Option 1: Deploy to Render.com (Recommended)

### Why Render?
- ✅ Free tier available
- ✅ Native Python support
- ✅ Easy GitHub integration
- ✅ Built-in SSL/TLS
- ✅ No cold starts

### Step 1: Automated Deployment

```bash
cd rag-backend
./deploy-render.sh
```

This script will:
1. Check prerequisites
2. Create `render.yaml` configuration
3. Provide deployment instructions
4. Guide you through Vercel configuration

### Step 2: Manual Render Setup

If you prefer manual setup:

1. **Go to Render Dashboard**
   - Visit: https://dashboard.render.com/select-repo
   - Connect your GitHub account

2. **Create Web Service**
   - Click "New +" → "Web Service"
   - Select your repository
   - Configure:
     - **Name**: `rag-chatbot-api`
     - **Region**: Oregon (or closest to Qdrant cluster)
     - **Branch**: `main`
     - **Root Directory**: `rag-backend`
     - **Runtime**: Python 3
     - **Build Command**: `pip install -r requirements.txt`
     - **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

3. **Add Environment Variables**
   - Go to Environment tab
   - Add all variables from "Required Environment Variables" section above
   - **Important**: Set `ALLOWED_ORIGINS` to include your Vercel URL

4. **Deploy**
   - Click "Create Web Service"
   - Wait 5-10 minutes for first deployment

5. **Note Your URL**
   - Example: `https://rag-chatbot-api.onrender.com`
   - Save this for Vercel configuration

### Step 3: Verify Backend Deployment

```bash
# Test health endpoint
curl https://rag-chatbot-api.onrender.com/api/health

# Expected response:
# {"status":"healthy","timestamp":"...","message":"Service is healthy"}
```

---

## Option 2: Deploy to Railway.app (Alternative)

### Why Railway?
- ✅ Simple CLI deployment
- ✅ Automatic HTTPS
- ✅ Built-in database options
- ✅ Great developer experience

### Step 1: Automated Deployment

```bash
cd rag-backend
./deploy-railway.sh
```

This script will:
1. Install Railway CLI (if needed)
2. Initialize project
3. Set environment variables from `.env`
4. Deploy backend
5. Create public domain
6. Test health endpoint

### Step 2: Manual Railway Setup

If you prefer manual setup:

1. **Install Railway CLI**
   ```bash
   npm install -g @railway/cli
   ```

2. **Login**
   ```bash
   railway login
   ```

3. **Initialize Project**
   ```bash
   cd rag-backend
   railway init
   ```

4. **Set Environment Variables**
   ```bash
   # Set each variable
   railway variables set COHERE_API_KEY=<key>
   railway variables set QDRANT_URL=<url>
   railway variables set QDRANT_API_KEY=<key>
   railway variables set DATABASE_URL=<url>
   railway variables set ADMIN_API_KEY=<key>
   railway variables set ALLOWED_ORIGINS=https://physical-ai-textbook-jet.vercel.app,http://localhost:3000
   railway variables set REQUEST_TIMEOUT_SECONDS=300
   ```

5. **Deploy**
   ```bash
   railway up
   ```

6. **Create Public Domain**
   ```bash
   railway domain
   ```

7. **Note Your URL**
   - Example: `https://your-app.railway.app`

### Step 3: Verify Backend Deployment

```bash
# Test health endpoint
curl https://your-app.railway.app/api/health
```

---

## Configure Vercel Frontend

After backend is deployed, connect frontend to backend:

### Option A: Automated Configuration

```bash
cd rag-backend
./configure-vercel.sh https://your-backend-url.com
```

This script will:
1. Test backend health
2. Set Vercel environment variable
3. Optionally trigger deployment

### Option B: Manual Vercel Configuration

1. **Go to Vercel Dashboard**
   - Visit: https://vercel.com/dashboard
   - Select project: `physical-ai-textbook-jet`

2. **Add Environment Variable**
   - Go to: Settings → Environment Variables
   - Click "Add New"
   - Configure:
     - **Name**: `REACT_APP_API_URL`
     - **Value**: `https://your-backend-url.com` (no trailing slash)
     - **Environments**: Production, Preview, Development (select all)
   - Click "Save"

3. **Redeploy Frontend**
   - Go to: Deployments tab
   - Click "..." on latest deployment
   - Click "Redeploy"
   - Wait 2-3 minutes

4. **Verify Environment Variable**
   - Check deployment logs for:
     ```
     REACT_APP_API_URL=https://your-backend-url.com
     ```

---

## Post-Deployment Testing

### 1. Backend Health Check

```bash
curl https://your-backend.com/api/health
```

**Expected:**
```json
{
  "status": "healthy",
  "timestamp": "2025-12-21T...",
  "message": "Service is healthy"
}
```

### 2. CORS Test

Open browser console on Vercel site:
```javascript
fetch('https://your-backend.com/api/health')
  .then(r => r.json())
  .then(console.log)
```

**Expected:** No CORS error, receives JSON response

### 3. Full Integration Test

1. **Open Frontend**
   - Go to: https://physical-ai-textbook-jet.vercel.app

2. **Test Query**
   - Type: "What is forward kinematics?"
   - Click "Ask"

3. **Verify**
   - ✅ No console errors (F12 → Console)
   - ✅ Answer appears within 3 seconds
   - ✅ Citations display with clickable links
   - ✅ Deep links navigate to textbook sections

4. **Test Error Handling**
   - Query: "What is quantum computing?" (out of scope)
   - Expected: "I cannot find sufficient information in the textbook..."

### 4. Performance Test

Open browser DevTools (F12) → Network tab:
- Submit query
- Check timing: Should be <3s total

### 5. Citation Deep Link Test

- Click a citation link
- Verify: Navigates to textbook section
- Verify: Paragraph is highlighted

---

## Ingestion (Admin Only)

### Option 1: Synchronous Ingestion (Simple)

```bash
curl -X POST https://your-backend.com/api/ingest \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: <your-admin-key>" \
  -d '{
    "book_path": "../textbook/docs",
    "chunk_size": 400,
    "chunk_overlap": 0.15
  }'
```

**Note:** May timeout for large books (>5 min). Use background ingestion instead.

### Option 2: Background Ingestion (Recommended)

```bash
# Start ingestion
curl -X POST https://your-backend.com/api/ingest/background \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: <your-admin-key>" \
  -d '{
    "book_path": "../textbook/docs"
  }'

# Response:
# {"task_id":"abc-123","status":"started","status_url":"/api/ingest/status/abc-123"}

# Check status
curl https://your-backend.com/api/ingest/status/abc-123

# Poll until status is "completed"
```

---

## Troubleshooting

### Issue: "Failed to fetch" or CORS errors

**Problem:** Frontend cannot reach backend or CORS blocking requests

**Solutions:**
1. **Check backend health:**
   ```bash
   curl https://your-backend.com/api/health
   ```

2. **Verify CORS configuration:**
   - Backend must have: `ALLOWED_ORIGINS=https://physical-ai-textbook-jet.vercel.app`
   - Check Render/Railway environment variables

3. **Verify Vercel env var:**
   - Should be: `REACT_APP_API_URL=https://your-backend.com`
   - No trailing slash

4. **Check browser console:**
   - Look for specific CORS error message
   - Verify request URL is correct

### Issue: 504 Gateway Timeout on ingestion

**Problem:** Ingestion takes too long

**Solutions:**
1. **Use background ingestion:**
   ```bash
   POST /api/ingest/background
   ```

2. **Increase timeout:**
   - Set `REQUEST_TIMEOUT_SECONDS=300` in backend env vars

3. **Check ingestion logs:**
   - Render: Dashboard → Logs
   - Railway: `railway logs`

### Issue: INTERNAL_ERROR on queries

**Problem:** Backend error during query processing

**Solutions:**
1. **Check backend logs:**
   - Render: Dashboard → Logs
   - Railway: `railway logs`

2. **Common causes:**
   - **Cohere API key missing/invalid**
     - Verify: `COHERE_API_KEY` is set correctly

   - **Qdrant connection failure**
     - Test: `curl $QDRANT_URL/health`
     - Verify: `QDRANT_API_KEY` is correct

   - **Database connection failure**
     - Verify: `DATABASE_URL` includes `?sslmode=require`
     - Test: `psql $DATABASE_URL -c "SELECT 1"`

3. **Test direct API call:**
   ```bash
   curl -X POST https://your-backend.com/api/query \
     -H "Content-Type: application/json" \
     -d '{"query_text":"What is robotics?","selected_text":null}'
   ```

### Issue: Slow response times (>3s)

**Problem:** Queries taking too long

**Solutions:**
1. **Check Qdrant cluster location:**
   - Should be in same region as backend (Oregon if using Render default)

2. **Optimize similarity threshold:**
   - Lower threshold = faster but less accurate
   - Try: `SIMILARITY_THRESHOLD=0.6`

3. **Check backend region:**
   - Render: Change region in settings
   - Railway: Redeploy in different region

### Issue: Vercel deployment fails

**Problem:** Frontend build failing

**Solutions:**
1. **Check build logs:**
   - Vercel Dashboard → Deployments → Click deployment → View logs

2. **Verify environment variable:**
   - Must be set BEFORE build
   - Redeploy after adding env var

3. **Common errors:**
   - `REACT_APP_API_URL is not defined`: Add env var and redeploy
   - TypeScript errors: Check for type issues in components

---

## Production Checklist

Use this checklist before going live:

### Backend Deployment
- [ ] Backend deployed to Render/Railway
- [ ] All environment variables configured
- [ ] `ALLOWED_ORIGINS` includes Vercel URL
- [ ] `REQUEST_TIMEOUT_SECONDS=300`
- [ ] Health endpoint responding: `GET /api/health`
- [ ] Backend logs show no startup errors

### Frontend Configuration
- [ ] `REACT_APP_API_URL` set in Vercel
- [ ] Frontend redeployed with new env var
- [ ] Build logs show correct API URL
- [ ] No localhost references in production build

### Database & Services
- [ ] Neon Postgres accessible from backend
- [ ] Qdrant cluster accessible from backend
- [ ] Cohere API key valid and working
- [ ] Database migrations run
- [ ] Qdrant collection initialized

### Ingestion
- [ ] Book content ingested (use background ingestion)
- [ ] 1713+ chunks in Qdrant
- [ ] Ingestion logs in database
- [ ] Sample queries return relevant chunks

### End-to-End Testing
- [ ] Frontend loads without errors
- [ ] Chatbot UI visible on homepage
- [ ] Query returns answer <3s
- [ ] Citations display correctly
- [ ] Deep links navigate to textbook sections
- [ ] Out-of-scope query rejected gracefully
- [ ] Error handling working (timeout, API failure)
- [ ] No CORS errors in browser console
- [ ] Performance: <500ms retrieval, <3s total

### Documentation
- [ ] Deployment steps documented
- [ ] Environment variables documented
- [ ] Troubleshooting guide available
- [ ] Admin ingestion instructions provided

---

## Monitoring & Maintenance

### Backend Monitoring

**Render:**
- Dashboard → Metrics: CPU, Memory, Response times
- Dashboard → Logs: Real-time application logs
- Alerts: Set up in Settings → Alerts

**Railway:**
- Dashboard → Metrics tab
- CLI: `railway logs --tail`
- Alerts: Configure in project settings

### Frontend Monitoring

**Vercel:**
- Analytics: Dashboard → Analytics
- Logs: Deployments → View Function Logs
- Performance: Real User Monitoring (RUM)

### Health Checks

Set up automated health checks:

```bash
# Cron job to ping health endpoint every 5 min
*/5 * * * * curl -f https://your-backend.com/api/health || echo "Backend down"
```

### Database Maintenance

**Neon Postgres:**
- Monitor storage usage
- Review query logs regularly
- Clean up old ingestion_logs periodically

**Qdrant:**
- Monitor vector count
- Check cluster health in dashboard
- Review indexing performance

---

## Scaling Considerations

### When to Upgrade

**Backend:**
- Free tier: 512MB RAM, shared CPU
- Upgrade when:
  - Response times >5s consistently
  - Out of memory errors in logs
  - >100 concurrent users

**Database:**
- Free tier: 0.5GB storage
- Upgrade when:
  - Query logs >400MB
  - Connection pool exhausted

**Qdrant:**
- Free tier: 1M vectors, 4GB RAM
- Upgrade when:
  - Approaching 1M chunks
  - Search latency >1s

### Performance Optimization

1. **Add caching:**
   - Redis for frequent queries
   - In-memory LRU cache

2. **Optimize embeddings:**
   - Batch larger (up to 96 chunks)
   - Use async embedding generation

3. **Database pooling:**
   - Increase max pool size
   - Add read replicas

4. **CDN:**
   - Serve static frontend via Vercel Edge
   - Cache API responses for common queries

---

## Rollback Procedure

If deployment fails, rollback:

### Backend Rollback

**Render:**
1. Dashboard → Deployments
2. Find last working deployment
3. Click "Rollback to this version"

**Railway:**
```bash
railway rollback
```

### Frontend Rollback

**Vercel:**
1. Dashboard → Deployments
2. Find last working deployment
3. Click "..." → "Promote to Production"

---

## Security Best Practices

1. **Never commit secrets**
   - Use `.env` (git-ignored)
   - Set env vars in hosting dashboard

2. **Rotate API keys**
   - ADMIN_API_KEY: Change quarterly
   - Cohere/Qdrant keys: Rotate if compromised

3. **Monitor usage**
   - Cohere API: Check usage dashboard
   - Qdrant: Monitor request counts
   - Neon: Track connection counts

4. **Rate limiting**
   - Already implemented for Cohere (100 calls/min)
   - Consider adding frontend rate limiting for queries

5. **HTTPS only**
   - Render/Railway: Automatic HTTPS
   - Vercel: Automatic HTTPS
   - Never use HTTP in production

---

## Support & Resources

**Documentation:**
- Backend README: `rag-backend/README.md`
- Architecture: `specs/001-rag-chatbot/plan.md`
- API Docs: `https://your-backend.com/api/docs`

**External Documentation:**
- Render: https://render.com/docs
- Railway: https://docs.railway.app
- Vercel: https://vercel.com/docs
- FastAPI: https://fastapi.tiangolo.com
- Cohere: https://docs.cohere.com
- Qdrant: https://qdrant.tech/documentation

**Getting Help:**
- Check troubleshooting section above
- Review backend logs
- Check browser console for frontend errors
- Test backend endpoints directly with curl

---

**Deployment guide complete.** Follow the steps above for a smooth production deployment.
