# ADR-009: Frontend Environment Variable Strategy

**Status:** Accepted
**Date:** 2025-12-21
**Deciders:** Development Team
**Related:** ADR-007 (Backend Deployment)

---

## Context

The frontend (Docusaurus React) needs to connect to different backend URLs depending on the environment:

- **Local development:** `http://localhost:8000`
- **Production:** `https://rag-chatbot-api.onrender.com` (or Railway, etc.)
- **Preview/Staging:** Potentially different URL

**Requirements:**
- No hardcoded backend URLs in code
- Different URLs for dev vs production
- Secure (no secrets exposed to client)
- Standard React/Docusaurus patterns
- Easy to configure in Vercel
- Works with both `npm run build` and `npm start`

---

## Decision

Use `REACT_APP_API_URL` environment variable with Vercel dashboard configuration and `.env.local` for local development.

---

## Rationale

### Why `REACT_APP_API_URL`?

**Pros:**
- ✅ **Standard React pattern:** `REACT_APP_` prefix is React convention
- ✅ **Build-time injection:** Value embedded during build, not runtime
- ✅ **Vercel native support:** First-class env var support per environment
- ✅ **No additional tooling:** Works with Create React App, Next.js, Docusaurus
- ✅ **Type-safe access:** `process.env.REACT_APP_API_URL` (string | undefined)
- ✅ **Git-ignored `.env.local`:** Local dev env vars never committed
- ✅ **Explicit naming:** Clear that it's the backend API URL

**Cons:**
- ⚠️ **Build-time only:** Cannot change URL without rebuild
- ⚠️ **No runtime config:** Cannot read from external config file
- ⚠️ **Exposed in client:** Visible in browser (but not secret - public API)

---

## Alternatives Considered

### 1. Hardcoded URLs with Conditional Logic

**Decision:** Rejected

**Approach:**
```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-api.onrender.com'
  : 'http://localhost:8000';
```

**Pros:**
- Simple, no env vars needed
- Works out of the box

**Cons:**
- ❌ **Not flexible:** Requires code change to update URL
- ❌ **No preview/staging:** Cannot distinguish preview from production
- ❌ **Hardcoded secrets:** URL embedded in code
- ❌ **No per-branch URLs:** Cannot use different URLs for feature branches

**Verdict:** Too inflexible, requires code changes for URL updates

---

### 2. Runtime Configuration File

**Decision:** Rejected (over-engineered)

**Approach:**
```typescript
// Fetch config at runtime
const config = await fetch('/config.json');
const API_BASE_URL = config.API_URL;
```

**Create `public/config.json`:**
```json
{
  "API_URL": "https://rag-chatbot-api.onrender.com"
}
```

**Pros:**
- Can change URL without rebuild
- Different configs per deployment

**Cons:**
- ❌ **Additional fetch:** Delays app startup
- ❌ **Complexity:** Requires async initialization
- ❌ **Error handling:** What if fetch fails?
- ❌ **Caching issues:** Browser may cache old config
- ❌ **Not standard:** Non-idiomatic for React/Docusaurus

**Verdict:** Too complex for simple URL configuration

---

### 3. Docusaurus Custom Config

**Decision:** Rejected (same issues as hardcoded)

**Approach:**
```typescript
// docusaurus.config.ts
customFields: {
  apiUrl: process.env.VERCEL_URL
    ? 'https://rag-chatbot-api.onrender.com'
    : 'http://localhost:8000',
},
```

**Access:**
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
const {siteConfig} = useDocusaurusContext();
const API_URL = siteConfig.customFields.apiUrl;
```

**Pros:**
- Docusaurus-native approach
- Centralized configuration

**Cons:**
- ❌ **Still hardcoded:** URL in config file, not env var
- ❌ **Not per-environment:** Cannot set different values in Vercel dashboard
- ❌ **Requires code change:** Must edit config file to change URL

**Verdict:** Doesn't solve the core problem of avoiding hardcoded URLs

---

### 4. Build-Time Script Replacement

**Decision:** Rejected (fragile)

**Approach:**
```bash
# build.sh
sed -i "s|API_URL_PLACEHOLDER|$REACT_APP_API_URL|g" build/**/*.js
```

**Pros:**
- Can inject values after build
- Flexible

**Cons:**
- ❌ **Fragile:** String replacement in minified JS
- ❌ **Non-standard:** Custom build script
- ❌ **Hard to maintain:** Breaks with code changes
- ❌ **Error-prone:** Easy to break with refactoring

**Verdict:** Too fragile and non-standard

---

## Implementation

### File Structure

```
textbook/
├── .env.local          # Git-ignored, local dev only
├── .env.example        # Committed, template for team
├── src/
│   └── components/
│       └── RAGChatbot/
│           └── index.tsx  # Uses process.env.REACT_APP_API_URL
└── .gitignore          # Includes .env.local
```

### Local Development Setup

**`.env.local` (git-ignored):**
```bash
# RAG Backend API URL
# For local development
REACT_APP_API_URL=http://localhost:8000
```

**`.env.example` (committed):**
```bash
# RAG Backend API URL
# For local development, use http://localhost:8000
# For production, set in Vercel Dashboard
REACT_APP_API_URL=http://localhost:8000
```

**`.gitignore`:**
```
.env.local
.env*.local
```

### Component Usage

**`src/components/RAGChatbot/index.tsx`:**
```typescript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const res = await fetch(`${API_BASE_URL}/api/query`, {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({query_text, selected_text}),
});
```

### Vercel Configuration

**Per-Environment Setup:**

1. **Production:**
   ```
   Name: REACT_APP_API_URL
   Value: https://rag-chatbot-api.onrender.com
   Environment: Production
   ```

2. **Preview:**
   ```
   Name: REACT_APP_API_URL
   Value: https://rag-chatbot-api-staging.onrender.com
   Environment: Preview
   ```

3. **Development:**
   ```
   Name: REACT_APP_API_URL
   Value: http://localhost:8000
   Environment: Development
   ```

### Verification

**Check build logs:**
```bash
# Vercel build logs should show:
REACT_APP_API_URL=https://rag-chatbot-api.onrender.com

# Verify in built JavaScript:
grep -r "rag-chatbot-api" .next/static/
# Should find the URL embedded in static files
```

---

## Consequences

### Positive

1. **Standard pattern:** Follows React ecosystem conventions
2. **Vercel-native:** First-class support in Vercel dashboard
3. **No code changes:** Change URL via dashboard, redeploy
4. **Per-environment:** Different URLs for prod/preview/dev
5. **Team-friendly:** `.env.example` documents required vars
6. **Secure:** `.env.local` git-ignored, secrets never committed

### Negative

1. **Build-time only:** Cannot change URL without redeploy
   - **Impact:** Minor - backend URL rarely changes
   - **Mitigation:** Redeploy takes <2 min on Vercel

2. **Not runtime-configurable:** Cannot use CDN config files
   - **Impact:** Low - not needed for this use case
   - **Mitigation:** None needed for hackathon scope

3. **Visible in client:** Backend URL visible in browser DevTools
   - **Impact:** None - backend URL is public anyway (not a secret)
   - **Note:** NEVER put secrets in REACT_APP_ vars (API keys, passwords)

### Trade-offs

| Aspect | Our Choice | Alternative | Rationale |
|--------|-----------|-------------|-----------|
| Flexibility | Build-time | Runtime config | Simpler, sufficient for stable backend URL |
| Tooling | Standard env vars | Custom solution | Less maintenance, team familiarity |
| Complexity | Low | High (runtime fetch) | Hackathon speed > flexibility |
| Security | Exposed (OK) | Same | Backend URL is public, not secret |

---

## Best Practices

### DO ✅

1. **Use `.env.local` for local development**
   ```bash
   # .env.local (never commit)
   REACT_APP_API_URL=http://localhost:8000
   ```

2. **Provide `.env.example` for team**
   ```bash
   # .env.example (commit this)
   REACT_APP_API_URL=http://localhost:8000  # Update for your setup
   ```

3. **Use fallback value in code**
   ```typescript
   const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
   ```

4. **Document in README**
   ```markdown
   ## Setup
   1. Copy `.env.example` to `.env.local`
   2. Update `REACT_APP_API_URL` if needed
   3. Run `npm start`
   ```

### DON'T ❌

1. **Never commit `.env.local`**
   - Add to `.gitignore`
   - Contains local-specific config

2. **Never put secrets in `REACT_APP_` vars**
   - Exposed to client
   - Only use for public values (API URLs, feature flags)

3. **Never hardcode URLs**
   - Always use env var
   - Enables environment-specific configuration

4. **Never use different var names**
   - Stick to one name: `REACT_APP_API_URL`
   - Avoid: `API_URL`, `BACKEND_URL`, `SERVER_URL` (inconsistent)

---

## Rollout Plan

### Phase 1: Local Development (Complete)

- [x] Create `.env.local` with localhost URL
- [x] Create `.env.example` template
- [x] Update `.gitignore`
- [x] Test locally: `npm start`

### Phase 2: Vercel Production (Required)

- [ ] Set `REACT_APP_API_URL` in Vercel dashboard
- [ ] Redeploy frontend
- [ ] Verify build logs show correct URL
- [ ] Test production: Query from deployed site

### Phase 3: Documentation (Required)

- [ ] Update README with env var setup
- [ ] Document in DEPLOYMENT.md
- [ ] Add troubleshooting section

---

## Monitoring & Validation

### Verification Checklist

```bash
# 1. Local development
echo $REACT_APP_API_URL  # Should be http://localhost:8000
npm start
# Open DevTools Network tab, check request goes to localhost:8000

# 2. Vercel build
# Check build logs for: REACT_APP_API_URL=https://...

# 3. Production
# Open https://physical-ai-textbook-jet.vercel.app
# DevTools → Network → Query request
# Verify: Request URL starts with production backend URL

# 4. Preview/Staging
# Same as production, but with preview URL
```

### Common Issues

**Issue:** "Failed to fetch" in production

**Diagnosis:**
```javascript
// In browser console on Vercel site:
console.log(process.env.REACT_APP_API_URL)
// If undefined: Env var not set in Vercel
// If localhost: Env var not set, using fallback
```

**Fix:**
1. Vercel Dashboard → Environment Variables
2. Add `REACT_APP_API_URL`
3. Redeploy

---

## Future Enhancements

### Short-term (1-2 months)

1. **Add environment indicator:**
   ```typescript
   // Show which backend is being used
   console.log(`[RAG Chatbot] API URL: ${API_BASE_URL}`);
   ```

2. **Add connection test:**
   ```typescript
   // Test backend health on app load
   useEffect(() => {
     fetch(`${API_BASE_URL}/api/health`)
       .then(r => r.json())
       .then(data => console.log('[Backend]', data.status))
       .catch(err => console.error('[Backend] Unreachable', err));
   }, []);
   ```

### Long-term (3-6 months)

1. **Multi-backend support:**
   ```typescript
   // Support fallback backends
   REACT_APP_API_URL=https://primary.com
   REACT_APP_API_URL_FALLBACK=https://backup.com
   ```

2. **Feature flags:**
   ```typescript
   // Enable/disable features per environment
   REACT_APP_ENABLE_STREAMING=true
   REACT_APP_ENABLE_CITATIONS=true
   ```

---

## References

- Create React App Env Vars: https://create-react-app.dev/docs/adding-custom-environment-variables/
- Vercel Environment Variables: https://vercel.com/docs/concepts/projects/environment-variables
- Docusaurus Environment Variables: https://docusaurus.io/docs/deployment#using-environment-variables

---

**Decision Owner:** Development Team
**Review Date:** 2026-01-21 (1 month post-deployment)
**Status:** ✅ Accepted and Implemented
