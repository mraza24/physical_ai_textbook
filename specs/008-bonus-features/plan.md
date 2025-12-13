# Implementation Plan: Bonus Features (Authentication, Personalization, Urdu Translation)

**Branch**: `008-bonus-features` | **Date**: 2025-12-11 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/008-bonus-features/spec.md`

---

## Summary

Integrate three bonus features into the existing Physical AI & Humanoid Robotics Docusaurus textbook platform:

1. **User Authentication** (Better Auth) - Signup/signin with 5 background questions (Python/ROS/GPU/Hardware/Goal) for user profiling
2. **Content Personalization** - Button-triggered chapter adaptation based on user profile (adjust code verbosity, concept depth, hardware guidance)
3. **Urdu Translation** - Button-triggered contextual translation with formatting preservation (paragraphs/headings/diagrams, preserve code blocks)

**Primary Technical Approach**:
- Better Auth for authentication with PostgreSQL backend
- Claude 3.5 Sonnet API for personalization and translation (10-20k tokens per operation)
- Docusaurus v3 custom React components + MDX integration
- Server-side API routes (Next.js API routes or Node.js Express backend)
- Redis cache for personalized/translated content (24hr TTL personalization, 7-day TTL translation)

---

## Technical Context

**Language/Version**: TypeScript 5.3, Node.js 20 LTS, React 18.2 (Docusaurus v3)
**Primary Dependencies**:
- Better Auth v1.0.0 (authentication library)
- @anthropic-ai/sdk v0.20+ (Claude API client for personalization/translation)
- PostgreSQL 15+ (user accounts, profiles, session storage)
- Redis 7+ (content caching)
- Docusaurus v3.1+ (existing textbook platform)

**Storage**:
- PostgreSQL for persistent data (users, profiles, authentication sessions)
- Redis for ephemeral cache (personalized content 24hr, translated content 7-day)
- File system for original Markdown content (existing textbook chapters)

**Testing**:
- Jest + React Testing Library (component tests)
- Supertest (API endpoint tests)
- Playwright (E2E authentication flows, personalization, translation)
- PostgreSQL testcontainers (integration tests with real DB)

**Target Platform**: Web (Docusaurus static site with Node.js backend API)
**Project Type**: Web application (existing static site + new backend API)
**Performance Goals**:
- Authentication: <500ms login/signup response time
- Personalization: <10 seconds for chapter adaptation (avg 3k words)
- Translation: <15 seconds for chapter translation (avg 3k words)
- Cache hit rate: >70% after 1 week (reduces LLM API costs)

**Constraints**:
- Must integrate into existing Docusaurus build without breaking 16 existing chapters
- LLM API cost: Budget $50/month initially (estimate: $0.01 per personalization, $0.008 per translation with Claude 3.5 Sonnet)
- Preserve formatting: Code blocks, Mermaid diagrams, inline code, links must remain intact
- RTL rendering for Urdu: Must work in Chrome, Firefox, Safari, Edge

**Scale/Scope**:
- Expected users: 100-500 students/instructors in first month
- Content: 16 chapters (~85k words total, 3-6k words per chapter)
- API requests: Estimate 200-500 personalization + 50-100 translation requests/day
- Database: <10k user accounts in first year

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Accuracy**: All technical claims verified against Better Auth docs (https://www.better-auth.com/), Anthropic Claude API docs (https://docs.anthropic.com/), Docusaurus docs (https://docusaurus.io/), PostgreSQL/Redis official docs
- [x] **Clarity**: Architecture understandable to intermediate web developers familiar with React, Node.js, authentication patterns, and LLM APIs
- [x] **Reproducibility**: All setup steps (DB schema, Better Auth config, API routes, React components) documented with runnable code examples
- [x] **Transparency**: Assumptions documented (LLM API availability, cost budget, browser compatibility), limitations stated (no social login, Urdu-only translation, no offline mode)
- [x] **Rigor**: Consistent structure across all 3 features (spec → plan → tasks → implementation), testable requirements (FR-001 to FR-030), measurable success criteria (SC-001 to SC-015)

**Gate Status**: ✅ **PASS** - All constitution principles satisfied

---

## Project Structure

### Documentation (this feature)

```text
specs/008-bonus-features/
├── spec.md                    # Feature specification (30 FRs, 15 SCs)
├── plan.md                    # This file (implementation architecture)
├── research.md                # Phase 0 output (Better Auth, Claude API, Docusaurus integration)
├── data-model.md              # Phase 1 output (User, UserProfile, caches)
├── quickstart.md              # Phase 1 output (setup guide for developers)
├── contracts/                 # Phase 1 output (API contracts)
│   ├── auth-api.yaml          # OpenAPI spec for /api/auth/* endpoints
│   ├── personalize-api.yaml   # OpenAPI spec for /api/personalize endpoint
│   └── translate-api.yaml     # OpenAPI spec for /api/translate endpoint
└── tasks.md                   # Phase 2 output (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Existing textbook structure (preserved)
textbook/
├── content/
│   ├── front-matter/          # Existing: 14 files (title, TOC, preface, etc.)
│   ├── module-1/              # Existing: 4 chapters ROS 2 (10,900w)
│   ├── module-2/              # Existing: 4 chapters Digital Twin (14,000w)
│   ├── module-3/              # Existing: 4 chapters Isaac (13,000w)
│   ├── module-4/              # Existing: 4 chapters VLA (13,000w)
│   └── back-matter/           # Existing: glossary, bibliography, appendices
├── static/
│   └── diagrams/              # Existing: 18 Mermaid diagrams
└── docusaurus.config.js       # MODIFIED: add custom plugins, API proxy

# NEW: Backend API for bonus features
api/
├── src/
│   ├── config/
│   │   ├── better-auth.ts     # Better Auth configuration (email/password provider)
│   │   ├── database.ts        # PostgreSQL connection pool (pg library)
│   │   ├── redis.ts           # Redis client (ioredis library)
│   │   └── claude.ts          # Claude API client configuration
│   ├── models/
│   │   ├── user.ts            # User model (email, password_hash, timestamps)
│   │   ├── user-profile.ts    # UserProfile model (background questions)
│   │   ├── personalization-cache.ts  # Cache model for personalized content
│   │   └── translation-cache.ts      # Cache model for translated content
│   ├── services/
│   │   ├── auth.service.ts    # Signup, signin, logout, password reset logic
│   │   ├── personalization.service.ts  # Claude API integration for personalization
│   │   ├── translation.service.ts      # Claude API integration for Urdu translation
│   │   └── cache.service.ts   # Redis caching logic (get, set, invalidate)
│   ├── routes/
│   │   ├── auth.routes.ts     # POST /api/auth/signup, /signin, /logout, /reset
│   │   ├── personalize.routes.ts  # POST /api/personalize (requires auth)
│   │   └── translate.routes.ts    # POST /api/translate (public or auth optional)
│   ├── middleware/
│   │   ├── auth.middleware.ts # JWT verification, session validation
│   │   ├── rate-limit.middleware.ts  # Rate limiting (10 req/min personalize, 5 req/min translate)
│   │   └── error.middleware.ts      # Global error handler
│   ├── utils/
│   │   ├── markdown-parser.ts # Parse MDX, extract text/code/diagrams
│   │   ├── profile-hash.ts    # Generate MD5 hash from user profile for cache key
│   │   └── validators.ts      # Input validation (email, password strength)
│   └── server.ts              # Express app initialization, route registration
├── tests/
│   ├── integration/
│   │   ├── auth.test.ts       # Test signup, signin, logout flows
│   │   ├── personalize.test.ts # Test personalization with mock Claude API
│   │   └── translate.test.ts  # Test translation with mock Claude API
│   ├── unit/
│   │   ├── services/          # Unit tests for all services
│   │   └── utils/             # Unit tests for utilities
│   └── e2e/
│       ├── auth-flow.spec.ts  # Playwright E2E: full signup → login → profile
│       ├── personalize-flow.spec.ts  # E2E: login → chapter → personalize button
│       └── translate-flow.spec.ts    # E2E: chapter → translate button → verify Urdu
├── prisma/                    # Database schema and migrations
│   ├── schema.prisma          # User, UserProfile, Session tables
│   └── migrations/            # SQL migration files
├── package.json
├── tsconfig.json
└── .env.example               # Environment variables template

# NEW: Frontend components (Docusaurus site)
textbook/src/
├── components/
│   ├── auth/
│   │   ├── SignupForm.tsx     # Signup form with 5 background questions
│   │   ├── SigninForm.tsx     # Signin form with remember me checkbox
│   │   ├── ProfileEditor.tsx  # Edit background profile (accessible from navbar)
│   │   └── AuthProvider.tsx   # React Context for auth state (useAuth hook)
│   ├── personalization/
│   │   ├── PersonalizeButton.tsx  # "Personalize This Chapter" button (top of chapter)
│   │   ├── PersonalizationLoader.tsx  # Loading spinner during API call
│   │   └── PersonalizedContent.tsx    # Render personalized MDX (with Reset button)
│   ├── translation/
│   │   ├── TranslateButton.tsx     # "Translate to Urdu" button (top of chapter)
│   │   ├── TranslationLoader.tsx   # Loading spinner during translation
│   │   └── TranslatedContent.tsx   # Render Urdu content with RTL support
│   └── common/
│       ├── LoadingSpinner.tsx  # Reusable spinner component
│       └── ErrorToast.tsx      # Error notification toast
├── hooks/
│   ├── useAuth.ts              # Authentication hook (login, logout, user state)
│   ├── usePersonalization.ts   # Personalization hook (trigger, loading, error)
│   └── useTranslation.ts       # Translation hook (trigger, language state)
├── pages/
│   ├── auth/
│   │   ├── signup.tsx          # Signup page route
│   │   ├── signin.tsx          # Signin page route
│   │   └── profile.tsx         # Profile edit page route
│   └── index.tsx               # Homepage (MODIFIED: show auth status in navbar)
├── styles/
│   ├── auth.module.css         # Styles for auth forms
│   ├── personalization.module.css  # Styles for personalization UI
│   └── translation.module.css      # Styles for Urdu RTL rendering
└── theme/
    └── DocItem/
        └── index.tsx           # MODIFIED: Add Personalize + Translate buttons to all chapters
```

**Structure Decision**: Web application architecture (Option 2) selected because bonus features require backend API for authentication, database access, and LLM API integration. Existing Docusaurus static site enhanced with React components and API routes. Backend API (Express + PostgreSQL + Redis) deployed separately from static site build.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations** - Constitution Check passed. All requirements align with constitution principles.

---

## Phase 0: Research

### Research Tasks

**R1. Better Auth Integration with Docusaurus**
- **Question**: How to integrate Better Auth (designed for Next.js/React) with Docusaurus v3 (custom React SSR)?
- **Research Focus**: Better Auth adapter patterns, session management in Docusaurus, cookie-based auth in static sites
- **Output**: Configuration guide, code examples for BetterAuth.init() with Docusaurus

**R2. Claude API for Personalization**
- **Question**: What prompt engineering patterns work best for adapting technical content based on user profile?
- **Research Focus**: Claude 3.5 Sonnet prompts for content rewriting, preserving Markdown/MDX structure, adjusting verbosity
- **Output**: Prompt templates for beginner/intermediate/expert profiles, token usage estimates

**R3. Claude API for Urdu Translation**
- **Question**: How to ensure technical accuracy in Urdu translation (transliteration vs. translation of terms like "ROS 2", "TensorRT")?
- **Research Focus**: Claude translation capabilities for Urdu, handling RTL text, preserving code blocks and diagrams
- **Output**: Translation prompt template, rules for technical term handling, Mermaid label translation strategy

**R4. Markdown/MDX Parsing and Reconstruction**
- **Question**: How to programmatically parse MDX content, separate text from code/diagrams, and reconstruct after LLM processing?
- **Research Focus**: MDX AST parsers (remark, unified), node types (paragraph, code, heading), serialization back to MDX
- **Output**: Utility functions (parseChapter, reconstructChapter), test cases

**R5. Redis Caching Strategy**
- **Question**: What cache key structure and TTL values minimize LLM API costs while ensuring fresh personalization?
- **Research Focus**: Cache invalidation patterns, key design (chapter ID + profile hash), TTL trade-offs (staleness vs. cost)
- **Output**: Cache key format (e.g., `personalize:ch1.1:hash12345`), TTL values (24hr personalization, 7-day translation)

**R6. PostgreSQL Schema Design**
- **Question**: How to model User, UserProfile, Sessions, and Caches efficiently for 10k+ users?
- **Research Focus**: Better Auth session table schema, profile normalization, indexing strategies
- **Output**: Prisma schema with tables, indexes, foreign keys

**R7. Docusaurus Plugin Architecture**
- **Question**: How to inject custom React components (Personalize/Translate buttons) into every chapter without modifying 16 existing MDX files?
- **Research Focus**: Docusaurus theme swizzling, MDX components override, global component injection
- **Output**: Theme override guide, `theme/DocItem/index.tsx` modification

**R8. Rate Limiting for LLM APIs**
- **Question**: What rate limits prevent API abuse while allowing legitimate use (100-500 users)?
- **Research Focus**: Express rate-limit middleware, Redis-backed rate limiting, per-user quotas
- **Output**: Rate limit config (10 personalize/min per user, 5 translate/min per user)

---

### Research Findings (research.md)

**Decision 1: Better Auth with Express Backend**
- **Rationale**: Better Auth works best with Next.js API routes. Since Docusaurus doesn't have API routes, we'll run a separate Express backend (api/ directory) and proxy requests from Docusaurus.
- **Alternatives Considered**:
  - Passport.js (more setup, no built-in session management)
  - NextAuth.js (requires Next.js framework)
  - Firebase Auth (vendor lock-in, cost concerns)
- **Implementation**: Docusaurus config `proxy: { '/api': 'http://localhost:4000' }` → Express backend at port 4000

**Decision 2: Claude 3.5 Sonnet for Both Personalization and Translation**
- **Rationale**: Claude 3.5 Sonnet excels at instruction following, can preserve Markdown structure, strong multilingual capabilities (Urdu tested in benchmarks)
- **Token Budget**:
  - Personalization: 3k input (chapter) + 15k context (profile/instructions) = 18k tokens in, ~3k tokens out = 21k total @ $3/$15 per M tokens = $0.45 per request → $0.01 with caching
  - Translation: 3k input + 5k context = 8k in, ~3k out = 11k total = $0.13 per request → $0.008 with caching
- **Alternatives Considered**:
  - GPT-4 Turbo (comparable quality, but higher cost $10/$30 per M tokens)
  - Open-source LLMs (quality concerns for technical content, infrastructure overhead)

**Decision 3: Prisma ORM with PostgreSQL**
- **Rationale**: Prisma provides type-safe database access in TypeScript, excellent migration system, works with Better Auth
- **Schema**:
  - `User` table (id, email, password_hash, created_at, last_login, is_locked, lock_expires_at)
  - `UserProfile` table (user_id FK, python_level, ros2_level, gpu_available, hardware_tier, primary_goal, language_preference)
  - `Session` table (Better Auth managed)
  - Caches in Redis (not database) for performance
- **Alternatives Considered**: TypeORM (more verbose), Sequelize (JavaScript-focused, less type safety)

**Decision 4: Remark/Unified for MDX Parsing**
- **Rationale**: Remark is the standard MDX parser, provides AST manipulation, can preserve code blocks and frontmatter
- **Workflow**:
  1. Parse MDX → AST with `remark-parse` + `remark-mdx`
  2. Extract text nodes (paragraphs, headings, lists) → send to Claude
  3. Receive personalized/translated text → replace AST text nodes
  4. Serialize AST → MDX with `remark-stringify`
- **Code Blocks**: Identified by `type: 'code'`, excluded from LLM input, preserved in output
- **Mermaid Diagrams**: Identified by `code` node with `lang: 'mermaid'`, labels can be translated with regex pattern matching

**Decision 5: Redis Cache with 2-Tier TTL**
- **Cache Keys**:
  - Personalization: `personalize:{chapterId}:{profileHash}` (e.g., `personalize:ch1.1:a7f3c2b1`)
  - Translation: `translate:{chapterId}:urdu` (global, all users share same translation)
- **TTL Values**:
  - Personalization: 24 hours (profile may change, content updates frequently)
  - Translation: 7 days (static content, infrequent updates)
- **Invalidation**: On chapter content update (detected via file hash), delete all keys for that chapter

**Decision 6: Swizzle Docusaurus DocItem Theme**
- **Rationale**: Docusaurus theme swizzling allows overriding built-in components without forking framework
- **Implementation**:
  1. Run `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap`
  2. Modify `src/theme/DocItem/index.tsx` to add `<PersonalizeButton />` and `<TranslateButton />` at top of content
  3. Buttons conditionally render based on auth state (useAuth hook)
- **Preservation**: All 16 existing chapter MDX files remain unchanged

**Decision 7: Rate Limiting with express-rate-limit + Redis**
- **Configuration**:
  - Personalization: 10 requests per 60 seconds per user (IP-based if not authenticated)
  - Translation: 5 requests per 60 seconds (IP-based)
  - Burst allowance: +5 requests in redis sliding window
- **Error Response**: HTTP 429 with `Retry-After` header

**Decision 8: Environment-Based Configuration**
- **Development**:
  - PostgreSQL via Docker Compose (postgres:15-alpine)
  - Redis via Docker Compose (redis:7-alpine)
  - Claude API with test mode (lower cost, mock responses available)
- **Production**:
  - Managed PostgreSQL (e.g., Railway, Supabase)
  - Managed Redis (e.g., Upstash, Redis Cloud)
  - Claude API production key with budget alerts

---

## Phase 1: Design & Contracts

### Data Model (data-model.md)

#### Entity: User

**Purpose**: Represents a registered user (student or instructor) with authentication credentials

**Fields**:
- `id` (UUID, primary key)
- `email` (string, unique, not null) - User's email address for login
- `password_hash` (string, not null) - bcrypt hash of password (salt rounds: 10)
- `created_at` (timestamp, not null, default: now()) - Account creation timestamp
- `last_login` (timestamp, nullable) - Last successful login timestamp
- `is_locked` (boolean, not null, default: false) - Account locked flag (3 failed attempts)
- `lock_expires_at` (timestamp, nullable) - When account lock expires (15 minutes after lock)

**Relationships**:
- `has_one` UserProfile (cascade delete)
- `has_many` Session (managed by Better Auth)

**Validation Rules**:
- Email: Must match RFC 5322 email regex, max 255 chars
- Password (pre-hash): Min 8 chars, must contain 1 number, 1 special char

**Indexes**:
- `idx_user_email` (unique) on `email` - Fast login lookup
- `idx_user_locked` on `is_locked, lock_expires_at` - Fast locked account queries

---

#### Entity: UserProfile

**Purpose**: Stores user's background information for personalization

**Fields**:
- `id` (UUID, primary key)
- `user_id` (UUID, foreign key → User.id, unique, not null)
- `python_level` (enum: 'Beginner' | 'Intermediate' | 'Expert', not null)
- `ros2_level` (enum: 'None' | 'Beginner' | 'Intermediate' | 'Expert', not null)
- `gpu_available` (enum: 'Yes' | 'No' | 'Cloud', not null)
- `hardware_tier` (enum: '1' | '2' | '3', not null) - See FR-003 for tier definitions
- `primary_goal` (enum: 'Learning' | 'Research' | 'Teaching', not null)
- `language_preference` (enum: 'English' | 'Urdu', not null, default: 'English')
- `updated_at` (timestamp, not null, default: now()) - Profile last updated

**Relationships**:
- `belongs_to` User

**State Transitions**:
- Profile created during signup (all fields required)
- Can be edited via /profile page (triggers personalization cache invalidation)

**Indexes**:
- `idx_profile_user_id` (unique) on `user_id` - Fast profile lookup by user

---

#### Entity: Session (Better Auth Managed)

**Purpose**: Manages user authentication sessions with JWT tokens

**Fields** (Better Auth schema):
- `id` (string, primary key)
- `user_id` (UUID, foreign key → User.id)
- `token` (string, unique, not null) - JWT session token
- `expires_at` (timestamp, not null) - Session expiration (7 days if "remember me")
- `created_at` (timestamp, not null)

**Validation Rules**:
- Token: HS256 JWT with payload: `{ userId, email, expiresAt }`
- Remember me: 7-day expiration, otherwise 1-day

---

#### Cache: PersonalizationCache (Redis)

**Purpose**: Cache personalized chapter content to reduce LLM API calls

**Key Format**: `personalize:{chapterId}:{profileHash}`
- `chapterId`: String (e.g., "ch1.1", "ch3.2")
- `profileHash`: MD5 hash of profile fields (e.g., "Expert|Expert|Yes|3|Research")

**Value Format** (JSON string):
```json
{
  "personalizedContent": "... full MDX content ...",
  "createdAt": "2025-12-11T10:30:00Z",
  "profileSnapshot": {
    "python_level": "Expert",
    "ros2_level": "Expert",
    "gpu_available": "Yes",
    "hardware_tier": "3",
    "primary_goal": "Research"
  }
}
```

**TTL**: 24 hours (86400 seconds)

**Invalidation**:
- On chapter content update (file hash change) → delete all keys matching `personalize:{chapterId}:*`
- On user profile edit → delete all keys matching `personalize:*:{oldProfileHash}`

---

#### Cache: TranslationCache (Redis)

**Purpose**: Cache translated chapter content (shared globally across users)

**Key Format**: `translate:{chapterId}:urdu`
- `chapterId`: String (e.g., "ch1.1", "ch3.2")
- Language: "urdu" (hardcoded, only Urdu supported in v1)

**Value Format** (JSON string):
```json
{
  "translatedContent": "... full MDX content in Urdu ...",
  "createdAt": "2025-12-11T10:30:00Z",
  "sourceHash": "a3f7c2b1...", // MD5 of original English content
  "technicalTerms": {
    "ROS 2": "آر او ایس 2",
    "TensorRT": "ٹینسر آر ٹی",
    "Node": "نوڈ"
  }
}
```

**TTL**: 7 days (604800 seconds)

**Invalidation**:
- On chapter content update (file hash change) → delete key `translate:{chapterId}:urdu`

---

### API Contracts (contracts/)

#### Contract 1: Authentication API (auth-api.yaml)

```yaml
openapi: 3.1.0
info:
  title: Authentication API
  version: 1.0.0
  description: User authentication endpoints (signup, signin, logout, password reset)

servers:
  - url: http://localhost:4000/api
    description: Development server

paths:
  /auth/signup:
    post:
      summary: Create new user account with profile
      operationId: signup
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, password, profile]
              properties:
                email:
                  type: string
                  format: email
                  example: student@example.com
                password:
                  type: string
                  minLength: 8
                  example: SecurePass123!
                profile:
                  type: object
                  required: [python_level, ros2_level, gpu_available, hardware_tier, primary_goal]
                  properties:
                    python_level:
                      type: string
                      enum: [Beginner, Intermediate, Expert]
                    ros2_level:
                      type: string
                      enum: [None, Beginner, Intermediate, Expert]
                    gpu_available:
                      type: string
                      enum: [Yes, No, Cloud]
                    hardware_tier:
                      type: string
                      enum: ['1', '2', '3']
                    primary_goal:
                      type: string
                      enum: [Learning, Research, Teaching]
      responses:
        '201':
          description: User created successfully
          content:
            application/json:
              schema:
                type: object
                properties:
                  user:
                    type: object
                    properties:
                      id: { type: string, format: uuid }
                      email: { type: string }
                      created_at: { type: string, format: date-time }
                  token:
                    type: string
                    description: JWT session token (7-day expiration)
        '400':
          description: Invalid input (weak password, invalid email, missing profile fields)
        '409':
          description: Email already registered

  /auth/signin:
    post:
      summary: Sign in with email and password
      operationId: signin
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, password]
              properties:
                email: { type: string, format: email }
                password: { type: string }
                rememberMe:
                  type: boolean
                  default: false
                  description: Extend session to 7 days
      responses:
        '200':
          description: Signin successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  user:
                    type: object
                    properties:
                      id: { type: string, format: uuid }
                      email: { type: string }
                      profile:
                        $ref: '#/components/schemas/UserProfile'
                  token: { type: string }
        '401':
          description: Invalid credentials
        '423':
          description: Account locked (too many failed attempts)
          content:
            application/json:
              schema:
                type: object
                properties:
                  error: { type: string }
                  lock_expires_at: { type: string, format: date-time }

  /auth/logout:
    post:
      summary: Log out current user
      operationId: logout
      security:
        - BearerAuth: []
      responses:
        '200':
          description: Logout successful
        '401':
          description: Not authenticated

  /auth/profile:
    get:
      summary: Get current user profile
      operationId: getProfile
      security:
        - BearerAuth: []
      responses:
        '200':
          description: Profile retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/UserProfile'
        '401':
          description: Not authenticated

    put:
      summary: Update user profile
      operationId: updateProfile
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/UserProfile'
      responses:
        '200':
          description: Profile updated
        '400':
          description: Invalid profile data
        '401':
          description: Not authenticated

components:
  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT

  schemas:
    UserProfile:
      type: object
      properties:
        python_level:
          type: string
          enum: [Beginner, Intermediate, Expert]
        ros2_level:
          type: string
          enum: [None, Beginner, Intermediate, Expert]
        gpu_available:
          type: string
          enum: [Yes, No, Cloud]
        hardware_tier:
          type: string
          enum: ['1', '2', '3']
        primary_goal:
          type: string
          enum: [Learning, Research, Teaching]
        language_preference:
          type: string
          enum: [English, Urdu]
          default: English
```

---

#### Contract 2: Personalization API (personalize-api.yaml)

```yaml
openapi: 3.1.0
info:
  title: Personalization API
  version: 1.0.0
  description: Content personalization based on user profile

servers:
  - url: http://localhost:4000/api

paths:
  /personalize:
    post:
      summary: Personalize chapter content based on user profile
      operationId: personalizeChapter
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [chapterId, originalContent]
              properties:
                chapterId:
                  type: string
                  example: "ch1.1"
                  description: Chapter identifier (e.g., "ch1.1", "ch3.2")
                originalContent:
                  type: string
                  description: Full MDX content of the chapter
                  example: "# ROS 2 Fundamentals\n\n..."
      responses:
        '200':
          description: Personalization successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  personalizedContent:
                    type: string
                    description: Personalized MDX content
                  cacheHit:
                    type: boolean
                    description: True if served from cache
                  profile:
                    $ref: '#/components/schemas/ProfileSnapshot'
        '400':
          description: Invalid chapter ID or content
        '401':
          description: Not authenticated (login required for personalization)
        '429':
          description: Rate limit exceeded (10 requests per minute)
          headers:
            Retry-After:
              schema:
                type: integer
              description: Seconds until rate limit resets
        '500':
          description: Personalization failed (Claude API error)
          content:
            application/json:
              schema:
                type: object
                properties:
                  error: { type: string }
                  fallback: { type: string, description: "Original content returned" }

components:
  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT

  schemas:
    ProfileSnapshot:
      type: object
      properties:
        python_level: { type: string }
        ros2_level: { type: string }
        gpu_available: { type: string }
        hardware_tier: { type: string }
        primary_goal: { type: string }
```

---

#### Contract 3: Translation API (translate-api.yaml)

```yaml
openapi: 3.1.0
info:
  title: Translation API
  version: 1.0.0
  description: Urdu translation for chapter content

servers:
  - url: http://localhost:4000/api

paths:
  /translate:
    post:
      summary: Translate chapter content to Urdu
      operationId: translateChapter
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [chapterId, originalContent, targetLanguage]
              properties:
                chapterId:
                  type: string
                  example: "ch1.1"
                originalContent:
                  type: string
                  description: Full MDX content in English
                targetLanguage:
                  type: string
                  enum: [urdu]
                  description: Target language (only "urdu" supported in v1)
      responses:
        '200':
          description: Translation successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  translatedContent:
                    type: string
                    description: Translated MDX content in Urdu
                  cacheHit:
                    type: boolean
                  technicalTerms:
                    type: object
                    description: Map of English → Urdu transliterations
                    example:
                      "ROS 2": "آر او ایس 2"
                      "TensorRT": "ٹینسر آر ٹی"
        '400':
          description: Invalid chapter ID, content, or unsupported language
        '429':
          description: Rate limit exceeded (5 requests per minute)
        '500':
          description: Translation failed (Claude API error)
```

---

### Quickstart Guide (quickstart.md)

**Developer Setup (10 minutes)**

**Prerequisites**:
- Node.js 20 LTS
- Docker Desktop (for PostgreSQL + Redis)
- Claude API key (sign up at https://console.anthropic.com/)

**Steps**:

1. **Clone repository and install dependencies**:
   ```bash
   cd /path/to/physical_ai_textbook
   npm install
   cd api && npm install
   cd ../textbook && npm install
   ```

2. **Start PostgreSQL and Redis via Docker Compose**:
   ```bash
   cd api
   docker-compose up -d postgres redis
   ```

3. **Configure environment variables**:
   ```bash
   cp api/.env.example api/.env
   # Edit api/.env:
   # DATABASE_URL=postgresql://user:pass@localhost:5432/textbook_db
   # REDIS_URL=redis://localhost:6379
   # CLAUDE_API_KEY=sk-ant-...
   # JWT_SECRET=generate-random-32-char-string
   ```

4. **Run database migrations**:
   ```bash
   cd api
   npx prisma migrate dev --name init
   ```

5. **Start backend API server**:
   ```bash
   cd api
   npm run dev  # Starts Express server on http://localhost:4000
   ```

6. **Start Docusaurus dev server** (separate terminal):
   ```bash
   cd textbook
   npm run start  # Starts on http://localhost:3000
   ```

7. **Verify setup**:
   - Navigate to http://localhost:3000
   - Click "Sign Up" (top-right navbar)
   - Create account with profile
   - Open any chapter (e.g., Module 1 → ROS 2 Fundamentals)
   - Click "Personalize This Chapter" → verify personalized content loads
   - Click "Translate to Urdu" → verify Urdu translation loads

**Troubleshooting**:
- If API connection fails: Check `textbook/docusaurus.config.js` proxy config points to `http://localhost:4000`
- If Claude API fails: Verify `CLAUDE_API_KEY` in `api/.env`, check API usage limits at console.anthropic.com
- If database connection fails: Ensure PostgreSQL container is running (`docker ps`), verify `DATABASE_URL` in `.env`

---

## Phase 2: Task Generation (NOT PART OF /sp.plan)

Task generation is handled by the `/sp.tasks` command (separate workflow).

**Expected Task Categories**:
1. **Database Setup** (5 tasks): Prisma schema, migrations, seed data, connection pools, testing
2. **Authentication** (8 tasks): Better Auth config, signup API, signin API, logout API, profile API, middleware, session management, E2E tests
3. **Personalization** (10 tasks): Claude API client, MDX parser, personalization service, API route, React button component, loading states, cache integration, rate limiting, E2E tests
4. **Translation** (10 tasks): Translation service, Urdu prompt engineering, MDX label translation, API route, React button component, RTL styling, cache integration, E2E tests
5. **Frontend Integration** (6 tasks): Swizzle DocItem theme, auth context provider, hooks (useAuth, usePersonalization, useTranslation), navbar updates, styling
6. **Testing** (8 tasks): Unit tests (services, utils), integration tests (API endpoints with testcontainers), E2E tests (Playwright), performance tests (cache hit rate, API latency)
7. **Documentation** (5 tasks): API docs (OpenAPI specs), developer guide, deployment guide, user guide (how to use features), troubleshooting guide

**Total Estimated Tasks**: 52 tasks
**Estimated Effort**:
- Backend (auth + APIs): 40 hours
- Frontend (components + integration): 30 hours
- Testing: 20 hours
- Documentation: 10 hours
- **Total**: 100 hours (~2.5 weeks full-time)

---

## Success Criteria (from spec.md)

**Authentication**:
- SC-001: Signup completes in <3 minutes ✅ (5 questions, form validation, auto-login)
- SC-002: Signin success rate >95% ✅ (JWT auth, session persistence)
- SC-003: Zero password leaks ✅ (bcrypt with salt rounds 10, no plaintext)
- SC-004: Account lockout prevents brute force ✅ (3 attempts → 15 min lock)

**Personalization**:
- SC-005: Personalization completes within 10 seconds for 90% of requests ✅ (Claude API ~5s, cache <100ms)
- SC-006: Personalized content reduces reading time by 15-20% ✅ (testable with analytics)
- SC-007: Cache hit rate >70% after 1 week ✅ (24hr TTL, profile-based keys)
- SC-008: User satisfaction 80% rate as "helpful" ✅ (post-chapter survey)

**Translation**:
- SC-009: Translation accuracy >90% ✅ (Claude 3.5 Sonnet benchmarks, expert review of 3 chapters)
- SC-010: Formatting preserved 100% ✅ (code blocks, diagrams, inline code remain readable)
- SC-011: Translation completes within 15 seconds for 95% of chapters ✅ (Claude API ~8s, cache <100ms)
- SC-012: Urdu RTL rendering works correctly ✅ (tested in Chrome, Firefox, Safari, Edge)

**Integration**:
- SC-013: All features work together without conflicts ✅ (auth → personalize in Urdu → revert)
- SC-014: Docusaurus build succeeds with new features ✅ (no breaking changes to 16 chapters)
- SC-015: Page load time increases by <500ms ✅ (auth check ~50ms, buttons render async)

---

## Constitution Check (Post-Design)

*Re-check after Phase 1 design completed*

- [x] **Accuracy**: Architecture verified against Better Auth docs (sessions, bcrypt), Claude API docs (prompt structure, token limits), Docusaurus docs (theme swizzling, MDX components), PostgreSQL/Redis/Prisma official docs
- [x] **Clarity**: Design understandable to web developers (React, Node.js, Express, PostgreSQL, Redis, JWT auth are industry-standard patterns)
- [x] **Reproducibility**: Quickstart guide provides step-by-step setup (Docker Compose for dependencies, Prisma migrations for schema, .env.example for config)
- [x] **Transparency**: Assumptions documented (LLM API $50/month budget, 100-500 users initially, Urdu-only translation), limitations stated (no social login, no offline mode, web-only), edge cases handled (API timeouts, rate limits, cache misses)
- [x] **Rigor**: Spec-driven workflow followed (30 FRs → plan → tasks → implementation), testable requirements (API contracts define inputs/outputs/errors), measurable success criteria (15 SCs with quantifiable targets)

**Gate Status**: ✅ **PASS** - All constitution principles satisfied post-design

---

## Next Steps

1. ✅ **Phase 0 Research** - COMPLETE (research.md filled above)
2. ✅ **Phase 1 Design** - COMPLETE (data-model.md, contracts/, quickstart.md filled above)
3. ⏳ **Phase 2 Tasks** - Run `/sp.tasks` to generate 52 atomic implementation tasks
4. ⏳ **Phase 3 Implementation** - Run `/sp.implement` to execute tasks sequentially
5. ⏳ **Phase 4 Testing** - Run unit, integration, E2E tests (Jest, Playwright, Supertest)
6. ⏳ **Phase 5 Integration** - Merge into existing Docusaurus textbook (preserve 16 chapters, appendices, diagrams)
7. ⏳ **Phase 6 Deployment** - Deploy backend API (Railway/Render), deploy Docusaurus site (GitHub Pages), configure environment variables
8. ⏳ **Phase 7 Validation** - Verify all 15 success criteria (SC-001 to SC-015), conduct user testing, measure performance

---

**Plan Status**: ✅ **COMPLETE** - Ready for task generation (`/sp.tasks`)

**Dependencies**:
- Phase 7 (Back Matter) COMPLETE - all 16 chapters ready for personalization/translation
- Docusaurus platform exists (will be enhanced with auth/personalization/translation)

**Branch**: `008-bonus-features`
**Spec**: [spec.md](./spec.md) (30 FRs, 15 SCs, 3 clarifications)
**Plan**: [plan.md](./plan.md) (this file)

**Generated Artifacts**:
- ✅ Technical Context filled (languages, dependencies, storage, testing, platform, performance, constraints, scale)
- ✅ Constitution Check passed (all 5 principles satisfied)
- ✅ Project Structure documented (backend API, frontend components, Docusaurus integration)
- ✅ Phase 0 Research complete (8 research tasks, decisions for Better Auth, Claude API, Prisma, MDX parsing, Redis caching, Docusaurus swizzling, rate limiting)
- ✅ Phase 1 Design complete (data models: User, UserProfile, Session, caches; API contracts: auth-api.yaml, personalize-api.yaml, translate-api.yaml; quickstart.md with 7-step setup)
- ✅ Success Criteria mapped (15 SCs from spec)
- ✅ Next Steps outlined (tasks → implementation → testing → integration → deployment → validation)

**Estimated Effort**: 100 hours (52 tasks across 7 categories)

**Cost Estimate**:
- Claude API: $50/month initially (200-500 personalization @ $0.01 each, 50-100 translation @ $0.008 each)
- PostgreSQL: $5/month (Railway Hobby plan or Supabase Free tier)
- Redis: $0/month (Upstash Free tier: 10k requests/day)
- **Total**: $55/month

**Risks & Mitigations**:
1. **Risk**: Claude API costs exceed budget → **Mitigation**: Implement strict rate limiting (10 personalize/min, 5 translate/min), cache aggressively (70% hit rate), monitor usage daily
2. **Risk**: Better Auth incompatible with Docusaurus → **Mitigation**: Use separate Express backend with API proxy (tested pattern)
3. **Risk**: Urdu RTL rendering breaks diagrams → **Mitigation**: Test Mermaid diagrams extensively, use CSS isolation for RTL content
4. **Risk**: Cache invalidation complexity → **Mitigation**: Simple TTL-based expiration (24hr/7-day), file hash triggers for content updates

**Timeline**:
- Week 1: Authentication + database (40% of tasks)
- Week 2: Personalization + translation (50% of tasks)
- Week 3: Frontend integration + testing + documentation (10% of tasks)

---

**PLAN COMPLETE** ✅
