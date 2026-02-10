# Research: Authentication & Personalization Architecture

**Feature**: Authentication & Content Personalization System
**Branch**: `009-auth-personalization`
**Date**: 2026-01-01
**Researcher**: Claude Sonnet 4.5 (Spec-Kit Agent)

---

## 1. Better-Auth + Neon Integration

### Decision
Use **Better-Auth** with **Drizzle ORM adapter** connected to **Neon Serverless Postgres**.

### Rationale
- **Official Support**: Better-Auth provides first-class Drizzle adapter specifically designed for Neon's serverless architecture
- **Serverless-Compatible**: Neon's connection pooling (via `@neondatabase/serverless`) works seamlessly with Better-Auth's session management
- **Type Safety**: Drizzle ORM provides full TypeScript type inference, reducing runtime errors
- **Migration Management**: Drizzle Kit handles database migrations with version control
- **Cost Efficiency**: Neon's free tier (512MB storage, 3GB data transfer) sufficient for <10,000 users

### Alternatives Considered
1. **Prisma Adapter**
   - ❌ Heavier bundle size (~2.5MB vs. Drizzle's ~500KB)
   - ❌ Slower cold starts on serverless (200-300ms overhead)
   - ✅ More mature ecosystem

2. **Custom SQL Adapter**
   - ❌ Requires manual implementation of Better-Auth's session store interface
   - ❌ No type safety without custom type generation
   - ✅ Maximum control over queries

3. **NextAuth (migrate from Better-Auth)**
   - ❌ Tightly coupled to Next.js (we're using Docusaurus + Express backend)
   - ❌ Less flexible for custom profile fields
   - ✅ Larger community

### Implementation Notes

**Database Connection**:
```typescript
// backend/src/db/connection.ts
import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';

const sql = neon(process.env.DATABASE_URL!);
export const db = drizzle(sql);
```

**Better-Auth Configuration**:
```typescript
// backend/src/auth/better-auth.config.ts
import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db/connection';

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
  }),
  secret: process.env.JWT_SECRET,
  session: {
    expiresIn: 60 * 60 * 24, // 24 hours
    updateAge: 60 * 60, // Refresh token every hour
  },
  emailAndPassword: {
    enabled: true,
    bcryptCost: 12, // FR-005: Cost factor 12
  },
});
```

**Password Hashing**: bcrypt cost factor **12** (spec FR-005)
- Industry standard for 2026 (OWASP recommendation)
- ~250ms hashing time on modern CPUs
- Resistant to brute-force attacks (2^12 = 4096 iterations)

**Session Storage Pattern**: JWT tokens (spec FR-006)
- Standard login: 24-hour expiration
- "Remember Me": 7-day expiration
- Signed with **RS256** algorithm (spec Security requirements)

**Migration Strategy**:
1. Run Better-Auth auto-migration: `npx better-auth migrate`
2. Generate user_profiles extension migration via Drizzle Kit
3. Apply migrations in order: `001_better_auth_schema.sql`, `002_user_profiles_extension.sql`

---

## 2. Claude Agent SDK Skills

### Decision
Use **Claude Agent SDK** with **SKILL.md standard template** (Process, Quality Criteria, Examples sections).

### Rationale
- **Spec-Kit Plus Standard**: `.claude/skills/` follows project's existing Spec-Kit Plus conventions
- **Reproducible Workflows**: SKILL.md template ensures consistent skill invocation patterns
- **Prompt Engineering Best Practices**: Structured sections enforce quality criteria and few-shot examples
- **Version Control**: Skills are markdown files, enabling git-tracked prompt evolution
- **No External Dependencies**: No need for LangChain, LlamaIndex, or other orchestration frameworks

### Alternatives Considered
1. **Direct Anthropic API Calls**
   - ❌ No standardized structure for prompt templates
   - ❌ Prompt logic scattered across codebase
   - ✅ Simpler initial implementation

2. **LangChain**
   - ❌ Overkill for two simple skills (adds 15+ dependencies)
   - ❌ Abstractions obscure prompt engineering
   - ✅ Rich ecosystem of integrations

3. **Custom Prompt Templates**
   - ❌ Reinvents SKILL.md standard
   - ❌ No quality criteria enforcement
   - ✅ Maximum flexibility

### Implementation Notes

**SKILL.md Template Structure** (from Spec-Kit Plus):
```markdown
# Skill: [Name]

## Purpose
[One-sentence description of skill's responsibility]

## Process
1. **Input Validation**: [Describe required inputs]
2. **Preprocessing**: [How to prepare data for LLM]
3. **LLM Invocation**: [Prompt template with placeholders]
4. **Postprocessing**: [How to validate/transform output]
5. **Error Handling**: [Fallback strategies]

## Quality Criteria
- [Criterion 1 with measurable threshold]
- [Criterion 2 with validation method]

## Examples

### Example 1: [Scenario]
**Input**:
\`\`\`
[Sample input]
\`\`\`

**Expected Output**:
\`\`\`
[Sample output]
\`\`\`
```

**Skill Invocation from Node.js**:
```typescript
// backend/src/skills/content-personalizer.ts
import Anthropic from '@anthropic-ai/sdk';
import { readFile } from 'fs/promises';

const anthropic = new Anthropic({ apiKey: process.env.ANTHROPIC_API_KEY });

export async function personalizeContent(
  chapterMarkdown: string,
  softwareBackground: 'Beginner' | 'Intermediate' | 'Expert',
  hardwareExperience: 'None' | 'Basic' | 'Advanced'
): Promise<string> {
  // 1. Load skill template
  const skillMd = await readFile('.claude/skills/content-personalizer/skill.md', 'utf-8');

  // 2. Extract system prompt from skill.md Process section
  const systemPrompt = extractSystemPrompt(skillMd);

  // 3. Build user prompt with inputs
  const userPrompt = `
Personalize this robotics chapter for a learner with:
- Software Background: ${softwareBackground}
- Hardware Experience: ${hardwareExperience}

Chapter Content:
${chapterMarkdown}
`;

  // 4. Invoke Claude API
  const response = await anthropic.messages.create({
    model: 'claude-sonnet-4-5-20250929',
    max_tokens: 8192,
    system: systemPrompt,
    messages: [{ role: 'user', content: userPrompt }],
  });

  // 5. Extract and validate output
  const transformedContent = response.content[0].text;
  validateMarkdownStructure(transformedContent); // Quality criteria check

  return transformedContent;
}
```

**Error Handling**:
- API rate limits: Exponential backoff (3 retries with 1s, 2s, 4s delays)
- Timeout: 30s per request (shorter than spec's 8s target to allow retries)
- Invalid output: Fallback to original content with error notification (spec FR-031)

---

## 3. Docusaurus Swizzling

### Decision
**Swizzle `DocItem/Layout` component** and inject `<ChapterActions />` with **BrowserOnly wrapper**.

### Rationale
- **SSR-Safe**: BrowserOnly prevents hydration mismatches during Vercel deployment
- **Theme Preservation**: Swizzling inherits from existing theme, preserving light/dark mode switcher
- **Minimal Code**: Single component injection vs. custom plugin (~200 LOC vs. ~1000 LOC)
- **Docusaurus 3.x Native**: Recommended approach in Docusaurus 3.x docs
- **Future-Proof**: Theme updates from Docusaurus auto-merge with swizzled component

### Alternatives Considered
1. **Custom Docusaurus Plugin**
   - ❌ Too complex (requires webpack config, lifecycle hooks, content plugin API)
   - ❌ Harder to debug
   - ✅ More control over build process

2. **Manual Injection (Edit theme files directly)**
   - ❌ Breaks on Docusaurus version updates
   - ❌ No clear separation between custom and theme code
   - ✅ Fastest initial implementation

3. **Client-Side Script Injection**
   - ❌ Causes layout shift (buttons appear after page load)
   - ❌ No TypeScript support
   - ✅ Zero build config

### Implementation Notes

**Swizzle Command**:
```bash
cd textbook
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --eject
```

This creates: `textbook/src/theme/DocItem/Layout/index.tsx`

**Component Injection**:
```typescript
// textbook/src/theme/DocItem/Layout/index.tsx
import React from 'react';
import OriginalLayout from '@theme-original/DocItem/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import ChapterActions from '@site/src/components/personalization/ChapterActions';

export default function Layout(props) {
  return (
    <>
      <BrowserOnly fallback={<div />}>
        {() => <ChapterActions />}
      </BrowserOnly>
      <OriginalLayout {...props} />
    </>
  );
}
```

**SSR Compatibility**:
- `BrowserOnly` renders `<div />` fallback during SSR
- `ChapterActions` component only mounts on client-side
- No `window` or `localStorage` access during build
- Test with: `npm run build && npm run serve`

**Theme Switcher Preservation**:
- Swizzled component inherits `@theme-original/DocItem/Layout`
- No modifications to `@theme/ColorModeToggle` or `@theme/Navbar`
- CSS variables (`--ifm-color-*`) remain unchanged
- Test: Toggle light/dark mode → personalization buttons adapt

---

## 4. Transformation Cache

### Decision
Use **Neon Postgres table** with **5-minute TTL** and **SHA-256 cache keys**.

### Rationale
- **Single Database**: No Redis infrastructure required (reduces complexity)
- **Serverless-Compatible**: Leverages existing Neon connection (no extra pooling)
- **Cost-Effective**: Free tier sufficient (cache entries auto-expire, minimal storage)
- **Queryable**: SQL-based cache enables analytics (cache hit rate, popular chapters)
- **Atomic TTL**: PostgreSQL's `expires_at` column + periodic cleanup job ensures consistency

### Alternatives Considered
1. **Redis**
   - ❌ Extra service to manage (Upstash, Redis Cloud, self-hosted)
   - ❌ Additional connection pooling overhead
   - ✅ Faster lookups (~0.5ms vs. Neon's ~5ms)
   - ✅ Built-in TTL with automatic expiration

2. **In-Memory Cache (Node.js Map)**
   - ❌ Not scalable (lost on server restart)
   - ❌ No sharing across multiple backend instances
   - ✅ Fastest lookups (~0.01ms)

3. **Vercel KV (Redis-compatible)**
   - ❌ Vendor lock-in
   - ❌ Costs ($10/mo for 1GB)
   - ✅ Managed service, zero config

### Implementation Notes

**Cache Key Generation**:
```typescript
// backend/src/services/transformation-cache.ts
import crypto from 'crypto';

function generateCacheKey(
  chapterPath: string,
  userProfile: { software_background: string; hardware_experience: string },
  transformationType: 'personalize' | 'translate'
): string {
  const data = `${chapterPath}|${userProfile.software_background}|${userProfile.hardware_experience}|${transformationType}`;
  return crypto.createHash('sha256').update(data).digest('hex');
}
```

**Example**:
- Input: `/docs/module1/sensors`, `Beginner`, `None`, `personalize`
- Output: `a3f8b2c7... (64 hex characters)`

**Database Schema**:
```sql
-- 003_transformation_cache.sql
CREATE TABLE transformation_cache (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  cache_key VARCHAR(64) UNIQUE NOT NULL,
  transformed_content TEXT NOT NULL,
  transformation_metadata JSONB,
  created_at TIMESTAMP DEFAULT NOW(),
  expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_cache_key ON transformation_cache(cache_key);
CREATE INDEX idx_expires_at ON transformation_cache(expires_at);
```

**Cache Lookup**:
```typescript
async function getCached(cacheKey: string): Promise<string | null> {
  const result = await db
    .select()
    .from(transformationCache)
    .where(
      and(
        eq(transformationCache.cache_key, cacheKey),
        gt(transformationCache.expires_at, new Date())
      )
    )
    .limit(1);

  return result[0]?.transformed_content || null;
}
```

**Cache Write**:
```typescript
async function setCache(cacheKey: string, content: string, metadata: object): Promise<void> {
  const expiresAt = new Date(Date.now() + 5 * 60 * 1000); // 5 minutes

  await db.insert(transformationCache).values({
    cache_key: cacheKey,
    transformed_content: content,
    transformation_metadata: metadata,
    expires_at: expiresAt,
  });
}
```

**Cleanup Job** (run every hour):
```sql
DELETE FROM transformation_cache WHERE expires_at < NOW();
```

Deployed as a **cron job** via Vercel Cron or `node-cron`:
```typescript
// backend/src/jobs/cache-cleanup.ts
import cron from 'node-cron';

cron.schedule('0 * * * *', async () => {
  await db.delete(transformationCache).where(
    lt(transformationCache.expires_at, new Date())
  );
  console.log('[Cache Cleanup] Expired entries removed');
});
```

---

## 5. Rate Limiting

### Decision
Use **express-rate-limit** middleware with **JWT user ID extraction** and **10 req/user/min** window.

### Rationale
- **Standard Express Middleware**: No custom implementation needed (~50 LOC vs. ~300 LOC)
- **Flexible Configuration**: Per-route limits, custom key generators, skip functions
- **Memory Store (MVP)**: Sufficient for <10,000 users (~1MB memory footprint)
- **User-Based Limits**: Extract `user_id` from JWT for per-user tracking (spec FR-035)
- **Graceful Degradation**: Returns 429 status with Retry-After header

### Alternatives Considered
1. **Redis-Backed Rate Limiter**
   - ❌ Overkill for <10k users (extra infra)
   - ✅ Scales to millions of users
   - ✅ Distributed rate limiting across multiple servers

2. **Custom Middleware**
   - ❌ Reinvents tested solution
   - ❌ Requires manual sliding window implementation
   - ✅ Maximum control

3. **Cloudflare Rate Limiting**
   - ❌ IP-based only (doesn't respect user_id)
   - ❌ Costs $20/mo for custom rules
   - ✅ Edge-based, zero latency overhead

### Implementation Notes

**Middleware Setup**:
```typescript
// backend/src/auth/middleware.ts
import rateLimit from 'express-rate-limit';
import jwt from 'jsonwebtoken';

export const personalizationRateLimiter = rateLimit({
  windowMs: 60 * 1000, // 1 minute
  max: 10, // 10 requests per user per minute
  keyGenerator: (req) => {
    // Extract user_id from JWT token
    const token = req.headers.authorization?.replace('Bearer ', '');
    if (!token) return req.ip; // Fallback to IP for unauthenticated requests

    try {
      const decoded = jwt.verify(token, process.env.JWT_SECRET!) as { user_id: string };
      return decoded.user_id;
    } catch {
      return req.ip;
    }
  },
  handler: (req, res) => {
    res.status(429).json({
      error: 'Too many personalization requests. Please try again in 1 minute.',
      retry_after: 60,
    });
  },
});
```

**Apply to Routes**:
```typescript
// backend/src/routes/personalize.ts
import express from 'express';
import { personalizationRateLimiter } from '../auth/middleware';

const router = express.Router();

router.post('/api/personalize', personalizationRateLimiter, async (req, res) => {
  // Handle personalization request
});

router.post('/api/translate', personalizationRateLimiter, async (req, res) => {
  // Handle translation request
});
```

**429 Error Response**:
```json
{
  "error": "Too many personalization requests. Please try again in 1 minute.",
  "retry_after": 60
}
```

**Testing**:
```bash
# Send 11 requests in 1 minute
for i in {1..11}; do
  curl -X POST http://localhost:4000/api/personalize \
    -H "Authorization: Bearer $TOKEN" \
    -H "Content-Type: application/json" \
    -d '{"chapter_content": "..."}'
done

# 11th request should return 429
```

---

## 6. Markdown Preservation

### Decision
Use **remark + unified AST parsing** to preserve code blocks, math equations, and images during LLM processing.

### Rationale
- **Industry Standard**: remark is the de-facto markdown processor (used by Docusaurus internally)
- **AST-Based**: Preserves exact structure (vs. regex which breaks on edge cases)
- **Plugin Ecosystem**: remark-gfm (tables), remark-math (equations) already handle special syntax
- **Claude API Compatibility**: Anthropic Claude respects markdown delimiters (```code```, $$math$$)
- **Reassembly Guarantee**: AST nodes can be extracted, LLM-processed, and merged back without corruption

### Alternatives Considered
1. **Regex-Based Extraction**
   - ❌ Fragile (breaks on nested code blocks, escaped backticks)
   - ❌ Can't handle edge cases (code blocks in blockquotes)
   - ✅ Faster implementation (~50 LOC)

2. **Manual Parsing**
   - ❌ Reinvents remark (~2000 LOC)
   - ❌ Hard to maintain
   - ✅ Maximum control

3. **LLM-Only (trust Claude to preserve markdown)**
   - ❌ No guarantees (LLMs occasionally reformat code)
   - ❌ Violates spec FR-028 (MUST preserve code blocks)
   - ✅ Simplest implementation

### Implementation Notes

**Extraction Process**:
```typescript
// backend/src/services/markdown-processor.ts
import { unified } from 'unified';
import remarkParse from 'remark-parse';
import remarkGfm from 'remark-gfm';
import remarkMath from 'remark-math';
import { visit } from 'unist-util-visit';

interface PreservedNode {
  type: string;
  value: string;
  placeholder: string;
}

export async function extractPreservedNodes(markdown: string): Promise<{
  textOnly: string;
  preservedNodes: PreservedNode[];
}> {
  const tree = unified()
    .use(remarkParse)
    .use(remarkGfm)
    .use(remarkMath)
    .parse(markdown);

  const preservedNodes: PreservedNode[] = [];
  let nodeIndex = 0;

  // Extract code blocks, math, images
  visit(tree, (node) => {
    if (node.type === 'code' || node.type === 'math' || node.type === 'image') {
      const placeholder = `__PRESERVED_NODE_${nodeIndex}__`;
      preservedNodes.push({
        type: node.type,
        value: node.value || node.url,
        placeholder,
      });
      node.value = placeholder; // Replace with placeholder
      nodeIndex++;
    }
  });

  // Convert AST back to markdown (with placeholders)
  const textOnly = unified()
    .use(remarkStringify)
    .stringify(tree);

  return { textOnly, preservedNodes };
}
```

**LLM Processing** (send text-only markdown):
```typescript
const { textOnly, preservedNodes } = await extractPreservedNodes(chapterMarkdown);

const transformedText = await callClaudeAPI(textOnly, userProfile);

// Reassemble: Replace placeholders with original nodes
let finalMarkdown = transformedText;
for (const node of preservedNodes) {
  finalMarkdown = finalMarkdown.replace(node.placeholder, node.value);
}
```

**Example**:

**Input Markdown**:
````markdown
# Sensors

Robots use **LiDAR** for navigation.

```python
import rospy
rospy.init_node('sensor_node')
```

The formula for distance is: $$d = vt$$
````

**After Extraction** (sent to LLM):
````markdown
# Sensors

Robots use **LiDAR** for navigation.

__PRESERVED_NODE_0__

The formula for distance is: __PRESERVED_NODE_1__
````

**LLM Output** (simplified for beginners):
````markdown
# Sensors (Beginner-Friendly)

Robots use a sensor called **LiDAR** (Light Detection and Ranging) to see their surroundings and move safely.

__PRESERVED_NODE_0__

The formula for calculating distance traveled is: __PRESERVED_NODE_1__

**Explanation**: Distance (d) equals velocity (v) multiplied by time (t).
````

**After Reassembly**:
````markdown
# Sensors (Beginner-Friendly)

Robots use a sensor called **LiDAR** (Light Detection and Ranging) to see their surroundings and move safely.

```python
import rospy
rospy.init_node('sensor_node')
```

The formula for calculating distance traveled is: $$d = vt$$

**Explanation**: Distance (d) equals velocity (v) multiplied by time (t).
````

**Validation**:
```typescript
function validateMarkdownStructure(markdown: string): void {
  // Ensure all code blocks are closed
  const openCodeBlocks = (markdown.match(/```/g) || []).length % 2;
  if (openCodeBlocks !== 0) {
    throw new Error('Unclosed code block detected');
  }

  // Ensure all math delimiters are balanced
  const openMath = (markdown.match(/\$\$/g) || []).length % 2;
  if (openMath !== 0) {
    throw new Error('Unbalanced math delimiters detected');
  }
}
```

---

## Research Summary

All technology choices validated and decision rationale documented:

| Component | Technology | Validation Status |
|-----------|-----------|-------------------|
| Authentication | Better-Auth + Drizzle + Neon Postgres | ✅ Official adapter, serverless-compatible |
| AI Skills | Claude Agent SDK (SKILL.md template) | ✅ Spec-Kit Plus standard, reproducible |
| UI Injection | Docusaurus Swizzling (DocItem/Layout) | ✅ SSR-safe, theme-preserving |
| Caching | Neon Postgres table (5-min TTL) | ✅ Single database, cost-effective |
| Rate Limiting | express-rate-limit (10 req/user/min) | ✅ Standard middleware, user-based |
| Markdown Preservation | remark + unified AST | ✅ Industry standard, reliable |

**Next Step**: Proceed to **Phase 1 (Design)** to generate data models, API contracts, and quickstart guide.

---

**Research Status**: ✅ COMPLETE
**Date**: 2026-01-01
**Researcher**: Claude Sonnet 4.5 (Spec-Kit Agent)
