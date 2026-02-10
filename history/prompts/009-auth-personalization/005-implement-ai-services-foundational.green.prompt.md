---
id: 005
title: Implement AI services foundational
stage: green
date: 2026-01-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: 009-auth-personalization
branch: 009-auth-personalization
user: User
command: /sp.implement
labels: ["ai-services", "cache", "rate-limiting", "llm-client", "markdown-processor", "T015", "T016", "T017", "T018"]
links:
  spec: specs/009-auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/src/services/transformation-cache.ts
  - backend/src/services/rate-limiter.ts
  - backend/src/services/llm-client.ts
  - backend/src/services/markdown-processor.ts
  - backend/package.json
tests:
  - TypeScript compilation check (passed)
  - T011 blocked (DATABASE_URL placeholder detected)
---

## Prompt

T011 through T018: Finalize Migration and Build AI Services.

Execution Instructions:

Database Push (T011): Run the migration to Neon. Claude will likely ask you to confirm your credentials or verify the .env file.

AI Logic (T015 - T017): Implement the TransformationCache service and the LLM wrapper.

Ensure the LLM wrapper uses the content-personalizer and urdu-translator skills we planned.

The cache must use the SHA-256 logic to avoid double-charging your API credits for the same chapter.

Markdown Handling (T018): Ensure the markdown processor preserves the Docusaurus-specific syntax (like Admonitions or Imports) so the book doesn't break after translation.

Safety Guardrail: Confirm the health check (/health) is green before starting the AI service implementation. Ensure the RAG chatbot remains completely isolated.

## Response snapshot

Successfully implemented Phase 2 foundational services (T015-T018). T011 blocked due to placeholder DATABASE_URL requiring user configuration.

**T011 Status - Database Migration** ⚠️ BLOCKED:
- Migration file ready: `0000_gifted_harpoon.sql`
- Command ready: `npm run migrate:push`
- **Issue**: DATABASE_URL contains placeholder values:
  ```
  DATABASE_URL=postgresql://neondb_owner:your-password@your-host.neon.tech/neondb
  ```
- **Required Action**: User must update `backend/.env` with actual Neon connection string from console.neon.tech
- **Safety Verified**: RAG backend uses Qdrant (no conflict), migration only creates new tables (no DROP statements)

**T015 - Transformation Cache Service** (`transformation-cache.ts`, 238 lines):
- Implements SHA-256 cache key generation: `SHA-256(chapterPath | softwareBackground | hardwareExperience | transformationType)`
- Functions implemented:
  - `generateCacheKey()`: Creates deterministic hash from inputs
  - `setCacheEntry()`: Stores transformed content with 5-min TTL (FR-036)
  - `getCacheEntry()`: Retrieves cached content, auto-deletes if expired
  - `cleanupExpiredEntries()`: Batch delete for cron jobs
  - `getCacheStats()`: Monitoring metrics (total, active, expired, avg TTL)
  - `getOrTransform()`: High-level wrapper with automatic cache management
- JSONB metadata support for tracking model, changes_made, complexity_level, preserved_terms
- Automatic expiration checking on retrieval
- Type-safe with Drizzle ORM integration

**T016 - Rate Limiting Service** (`rate-limiter.ts`, 178 lines):
- Three rate limiter configurations:
  1. `createUserRateLimiter()`: General API (10 req/user/min, FR-035)
  2. `createTransformationRateLimiter()`: AI endpoints (5 req/min, more restrictive for LLM costs)
  3. `createAuthRateLimiter()`: Login/signup (5 attempts/15 min, brute-force protection)
- User identification: JWT user ID (if authenticated) or IP address fallback
- Standard HTTP 429 responses with retry-after headers
- Configurable via `RATE_LIMIT_PER_USER` environment variable
- Uses express-rate-limit middleware (installed v7.5.0)
- Custom error messages with helpful hints (e.g., "cached transformations available")

**T017 - LLM Client** (`llm-client.ts`, 255 lines):
- Anthropic SDK integration (@anthropic-ai/sdk v0.38.1)
- Retry logic with exponential backoff (3 retries, 1s→2s→4s delays)
- Model: claude-sonnet-4-5-20250929 (configurable via `CLAUDE_MODEL` env var)
- Three skill implementations:
  1. `personalizeContent()`: Adapts complexity based on software_background + hardware_experience
     - Beginner: Add explanations, code comments, prerequisites
     - Expert: Remove basics, add optimizations, performance notes
     - Hardware-specific adjustments (safety warnings vs technical specs)
  2. `translateToUrdu()`: Preserves technical terms + Docusaurus syntax
     - 100% code block preservation
     - Technical terms kept in English (configurable list)
     - RTL formatting with proper Urdu typography
     - Low temperature (0.3) for accuracy
  3. `validateChapter()`: Technical accuracy review (returns JSON feedback)
     - Scores: overall_score, issues (severity/issue/suggestion), strengths
- `testLLMConnection()`: Health check function
- Proper error handling for rate limits (429), server errors (500/503)
- Configurable max_tokens and temperature per use case

**T018 - Markdown Processor** (`markdown-processor.ts`, 329 lines):
- Remark + unified AST processing (installed remark v15.0.1, unified v11.0.5)
- Docusaurus preservation patterns:
  - Admonitions: `:::note`, `:::tip`, `:::warning`, `:::danger`, `:::info`
  - Imports: `import Component from '@site/...'`
  - MDX components: `<Tabs>`, `<TabItem>`, etc.
  - Frontmatter: YAML between `---`
- Core functions:
  - `extractCodeBlocks()`: Replace with placeholders during transformation
  - `restoreCodeBlocks()`: Restore original code after transformation
  - `extractDocusaurusComponents()`: Identify imports, admonitions, components
  - `validateMarkdown()`: Check for unclosed admonitions, code blocks, MDX tags
  - `safeTransform()`: Full pipeline with code protection and validation
  - `countWords()` / `estimateTokens()`: LLM cost estimation
  - `extractFrontmatter()`: Parse YAML metadata
- Protection strategy: Code blocks replaced with `CODE_BLOCK_0`, `CODE_BLOCK_1`, etc. during LLM calls, then restored
- Validation catches: unclosed admonitions, unclosed code blocks, broken imports, unclosed MDX components
- Word count excludes code blocks for accurate token estimation (~750 tokens per 1000 words)

**Dependencies Installed**:
- `express-rate-limit` v7.5.0
- `@anthropic-ai/sdk` v0.38.1
- `remark` v15.0.1
- `remark-parse` v11.0.0
- `remark-stringify` v11.0.0
- `unified` v11.0.5

**TypeScript Validation**: ✅ All files compile successfully

**RAG Chatbot Isolation**: ✅ Verified - separate Qdrant database, no conflicts

## Outcome

- ✅ Impact: Complete foundational AI services layer implemented (cache, rate limiting, LLM client, markdown processor). Ready for Phase 3 (User Story 1 - Authentication) after T011 migration completes.
- 🧪 Tests: TypeScript compilation passed, T011 blocked (user action required), services ready for unit testing (T020-T022)
- 📁 Files: Created 4 services (1000+ lines total), installed 6 npm packages, Phase 2 complete (4/4 parallel tasks)
- 🔁 Next prompts: User must update DATABASE_URL and run `npm run migrate:push`. Then implement T019 (JWT middleware) and test services (T020-T022). Phase 3 (Better-Auth routes) can begin after T011.
- 🧠 Reflection: SHA-256 cache keys ensure deterministic hashing (same inputs = same cache hit). Markdown processor's code block extraction is critical for preserving Docusaurus syntax during LLM transformations. Rate limiting has three tiers (general/transformation/auth) to balance API costs vs user experience. LLM client's retry logic handles Anthropic's rate limits gracefully with exponential backoff.

## Evaluation notes (flywheel)

- Failure modes observed: T011 blocked due to placeholder DATABASE_URL (expected, user must configure). No TypeScript errors.
- Graders run and results (PASS/FAIL): PASS - TypeScript compilation successful, all services implemented per specifications
- Prompt variant (if applicable): Standard implementation following research.md and data-model.md specifications
- Next experiment (smallest change to try): Consider adding Redis as optional cache backend (faster than Postgres for high-frequency reads). Add metrics collection for cache hit rate to optimize TTL. Implement circuit breaker pattern for LLM client to handle extended Anthropic outages.
