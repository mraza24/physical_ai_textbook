# Implementation Plan: Authentication & Content Personalization System

**Branch**: `009-auth-personalization` | **Date**: 2026-01-01 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-auth-personalization/spec.md`

## Summary

Integrate Better-Auth with Neon Serverless Postgres for user authentication and profiling, implement AI-powered content personalization and Urdu translation via Claude Agent SDK skills, and enhance the UI with cyber-physical aesthetics (neon borders, glassmorphism). The system preserves the existing RAG chatbot functionality while adding adaptive learning features based on user technical background.

## Technical Context

**Language/Version**: TypeScript 5.6.2 (Frontend), Python 3.11+ (Backend/Skills)
**Primary Dependencies**:
- Frontend: React 19, Docusaurus 3.9.2, Better-Auth SDK
- Backend: Better-Auth Server, Neon Serverless Postgres, Drizzle ORM
- AI: Anthropic Claude API (Sonnet 3.5), Claude Agent SDK

**Storage**: Neon Serverless Postgres (tables: `user`, `session`, `user_profiles`, `transformation_cache`)
**Testing**: Jest + React Testing Library (Frontend), Pytest (Backend/Skills)
**Target Platform**: Web (Vercel deployment), SSR-compatible
**Project Type**: Web (existing Docusaurus frontend + new Node.js backend microservice)
**Performance Goals**:
- Authentication: < 500ms signup/login
- Personalization: < 8s for 2000-3000 word chapters
- Translation: < 10s for 2000-3000 word chapters
- DB queries: < 100ms

**Constraints**:
- MUST NOT modify existing RAG chatbot (src/components/Chatbot/, custom.css chatbot styles)
- MUST reuse existing AuthProvider.tsx context structure
- MUST maintain Docusaurus SSR compatibility (use BrowserOnly for client-only code)
- MUST preserve existing theme switcher (light/dark mode)
- Session-based personalization only (no permanent storage of transformed content)

**Scale/Scope**:
- Initial release: < 10,000 users
- ~50 chapters requiring personalization
- 2 AI skills (content-personalizer, urdu-translator) + 1 research-validator subagent
- 3 new API endpoints (/api/personalize, /api/translate, /api/auth/*)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **Accuracy**: All technical, conceptual, and architectural claims must be verified.
  - ✓ Better-Auth + Neon Postgres compatibility verified in Better-Auth docs
  - ✓ Claude Agent SDK skill structure follows official SKILL.md template
  - ✓ Existing AuthProvider.tsx analyzed for integration points
- [x] **Clarity**: Content must be understandable to the target audience.
  - ✓ Spec includes clear user stories with Given/When/Then scenarios
  - ✓ Technical requirements separated from business requirements
- [x] **Reproducibility**: All instructions, code, and workflows must be reproducible.
  - ✓ All setup steps documented in quickstart.md (Phase 1 output)
  - ✓ Database schema defined in data-model.md with migrations
- [x] **Transparency**: All major decisions, assumptions, and limitations must be documented.
  - ✓ Spec includes Assumptions section (Better-Auth compatibility, LLM API, Neon free tier)
  - ✓ Risks documented with mitigations (LLM costs, auth integration, translation quality)
- [x] **Rigor**: Every chapter must maintain consistent structure, depth, and technical correctness.
  - ✓ Skill templates follow SKILL.md standard (Process, Quality Criteria, Examples)
  - ✓ API contracts follow OpenAPI 3.0 specification

## Project Structure

### Documentation (this feature)

```text
specs/009-auth-personalization/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── personalize.openapi.yaml
│   ├── translate.openapi.yaml
│   └── auth.openapi.yaml
├── checklists/          # Quality validation
│   └── requirements.md  # Spec validation checklist (COMPLETE)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# ENHANCED (existing Docusaurus project)
textbook/
├── src/
│   ├── components/
│   │   ├── Chatbot/                    # ✅ EXISTING - DO NOT MODIFY
│   │   │   └── index.js
│   │   ├── auth/                        # ✅ EXISTING - ENHANCE
│   │   │   ├── SignupForm.tsx           # MODIFY: Add software_background, hardware_experience fields
│   │   │   ├── LoginForm.tsx            # ✅ EXISTING - NO CHANGES
│   │   │   └── ProfileForm.tsx          # MODIFY: Add new profile fields
│   │   ├── personalization/             # ✅ EXISTING - ENHANCE
│   │   │   └── ChapterActions.tsx       # CREATE: "Personalize Chapter" + "Translate to Urdu" buttons
│   │   └── translation/                 # ✅ EXISTING - ENHANCE
│   │       └── UrduToggle.tsx           # CREATE: Toggle button component
│   ├── contexts/
│   │   └── AuthProvider.tsx             # ✅ MODIFY: Extend UserProfile interface, integrate Better-Auth
│   ├── hooks/
│   │   ├── usePersonalization.ts        # CREATE: Hook for personalization API calls
│   │   └── useTranslation.ts            # CREATE: Hook for translation API calls
│   ├── css/
│   │   └── custom.css                   # ✅ MODIFY: Add neon borders, glassmorphism (preserve chatbot styles)
│   └── theme/
│       ├── Root.js                      # ✅ EXISTING - DO NOT MODIFY
│       └── DocItem/                     # CREATE: Inject ChapterActions into doc pages
│           └── Layout/index.tsx         # CREATE: Docusaurus swizzle wrapper
├── .env.local                           # ✅ MODIFY: Add DATABASE_URL, ANTHROPIC_API_KEY
└── package.json                         # ✅ MODIFY: Add better-auth SDK dependencies

# NEW (Backend microservice for auth + AI transformations)
backend/
├── src/
│   ├── auth/
│   │   ├── better-auth.config.ts        # Better-Auth configuration with Neon DB
│   │   ├── routes.ts                    # /api/auth/signup, /signin, /logout, /profile
│   │   └── middleware.ts                # JWT validation middleware
│   ├── skills/
│   │   ├── content-personalizer.ts      # Wrapper to invoke Claude Agent SDK skill
│   │   ├── urdu-translator.ts           # Wrapper to invoke Claude Agent SDK skill
│   │   └── research-validator.ts        # Wrapper to invoke Claude Agent SDK subagent
│   ├── services/
│   │   ├── transformation-cache.ts      # Redis/Neon-based caching service (5-min TTL)
│   │   ├── rate-limiter.ts              # Rate limiting service (10 req/user/min)
│   │   └── llm-client.ts                # Anthropic Claude API client
│   ├── routes/
│   │   ├── personalize.ts               # POST /api/personalize
│   │   └── translate.ts                 # POST /api/translate
│   ├── db/
│   │   ├── schema.ts                    # Drizzle ORM schema (user_profiles, transformation_cache)
│   │   └── migrations/                  # Database migrations
│   └── index.ts                         # Express/Fastify server entry point
├── .env                                 # DATABASE_URL, ANTHROPIC_API_KEY, JWT_SECRET
└── package.json

# NEW (Claude Agent SDK Skills)
.claude/
└── skills/
    ├── content-personalizer/
    │   └── skill.md                     # SKILL.md standard: Process, Quality Criteria, Examples
    ├── urdu-translator/
    │   └── skill.md                     # SKILL.md standard: Process, Quality Criteria, Examples
    └── research-validator/
        └── skill.md                     # SKILL.md standard: Process, Quality Criteria, Examples
```

**Structure Decision**:

This feature requires a **web application architecture** (Option 2 from template) with:

1. **Enhanced Frontend** (existing `textbook/`): Extend existing Docusaurus project with:
   - Modified auth forms (signup with profile fields)
   - New chapter action buttons (personalize, translate)
   - Enhanced CSS for cyber-physical aesthetic
   - Reuse existing `AuthProvider.tsx` context (extend, don't replace)

2. **New Backend Microservice** (`backend/`): Separate Node.js service for:
   - Better-Auth integration with Neon DB
   - AI transformation API endpoints (/api/personalize, /api/translate)
   - Caching and rate limiting
   - JWT token validation

3. **Claude Agent SDK Skills** (`.claude/skills/`): Three skills following SKILL.md template:
   - `content-personalizer`: Adapts chapter complexity based on user profile
   - `urdu-translator`: Translates chapters while preserving technical terms
   - `research-validator`: Validates chapter technical accuracy (P4 priority)

**Why This Structure**:
- **Separation of Concerns**: Auth/AI logic isolated from Docusaurus frontend
- **Reuse Existing Assets**: Leverages existing `AuthProvider.tsx`, theme structure, and CSS patterns
- **Scalability**: Backend can scale independently from static Docusaurus build
- **Safety**: RAG chatbot (`src/components/Chatbot/`) remains untouched in separate directory
- **Maintainability**: Skills in `.claude/skills/` follow Spec-Kit Plus conventions

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Separate backend microservice instead of Docusaurus API routes | Better-Auth requires server-side runtime with database connections; Docusaurus is primarily static site generator with limited server capabilities | Docusaurus API routes are experimental and lack full Express/Fastify middleware support needed for Better-Auth integration |
| Session-based personalization (no permanent storage) | Reduces database costs and complexity for MVP; transformations are dynamic based on evolving LLM models | Permanent storage would require user content versioning, migration strategies for LLM updates, and 10x database storage costs |
| Three separate skills instead of monolithic AI service | Each skill has distinct inputs, outputs, and quality criteria; allows independent testing and iteration | Monolithic service would mix concerns (personalization vs. translation vs. validation), making testing and maintenance harder |

---

## Phase 0: Research & Technology Validation

**Objective**: Resolve all "NEEDS CLARIFICATION" items from Technical Context and validate technology choices.

### Research Tasks

1. **Better-Auth + Neon Integration**
   - Verify Better-Auth Drizzle adapter compatibility with Neon Postgres
   - Research session storage patterns (JWT vs. database sessions)
   - Document bcrypt cost factor recommendations for password hashing
   - **Output**: `research.md` Section 1

2. **Claude Agent SDK Skill Structure**
   - Review official SKILL.md template from Spec-Kit Plus docs
   - Research best practices for skill inputs/outputs (markdown → markdown transformations)
   - Document skill invocation patterns from Node.js backend
   - **Output**: `research.md` Section 2

3. **Docusaurus Swizzling for Chapter Actions**
   - Research Docusaurus 3.x swizzling capabilities for `DocItem/Layout`
   - Verify SSR compatibility for dynamic button injection
   - Document theme override patterns that preserve existing theme switcher
   - **Output**: `research.md` Section 3

4. **Transformation Caching Strategy**
   - Compare Neon Postgres vs. Redis for 5-minute TTL cache
   - Research cache key generation (hash of content + user profile)
   - Document cache invalidation patterns
   - **Output**: `research.md` Section 4

5. **Rate Limiting Implementation**
   - Research Express/Fastify rate limiting middleware options
   - Document 10 req/user/min throttling patterns
   - Verify JWT-based user identification for rate limits
   - **Output**: `research.md` Section 5

6. **Markdown Preservation During AI Transformation**
   - Research markdown AST parsing libraries (remark, unified)
   - Document strategies to preserve code blocks, equations, images during LLM processing
   - Validate Claude API's markdown handling capabilities
   - **Output**: `research.md` Section 6

### Research Deliverable

**File**: `specs/009-auth-personalization/research.md`

**Structure**:
```markdown
# Research: Authentication & Personalization Architecture

## 1. Better-Auth + Neon Integration
### Decision: [Drizzle adapter with Neon Postgres]
### Rationale: [Better-Auth official Neon adapter, serverless-compatible]
### Alternatives Considered: [Prisma adapter (heavier), custom SQL adapter (more work)]
### Implementation Notes: [Connection pooling, migration strategy]

## 2. Claude Agent SDK Skills
### Decision: [SKILL.md template with Process/Quality/Examples sections]
### Rationale: [Spec-Kit Plus standard, enables reproducible AI workflows]
### Alternatives Considered: [Direct API calls (no structure), LangChain (overkill)]
### Implementation Notes: [Skill invocation via Node.js subprocess, error handling]

## 3. Docusaurus Swizzling
### Decision: [Swizzle DocItem/Layout component with BrowserOnly wrapper]
### Rationale: [SSR-safe, preserves theme inheritance, minimal code]
### Alternatives Considered: [Custom plugin (too complex), manual injection (breaks updates)]
### Implementation Notes: [Wrap ChapterActions in BrowserOnly, test build output]

## 4. Transformation Cache
### Decision: [Neon Postgres table with 5-min TTL, indexed by cache_key]
### Rationale: [Single database, no Redis infra, leverages existing connection]
### Alternatives Considered: [Redis (extra service), in-memory (not scalable)]
### Implementation Notes: [Periodic cleanup job, hash cache key with SHA-256]

## 5. Rate Limiting
### Decision: [express-rate-limit middleware with JWT user ID extraction]
### Rationale: [Standard Express middleware, flexible config, memory store for MVP]
### Alternatives Considered: [Redis-backed (overkill for <10k users), custom middleware]
### Implementation Notes: [10 req/user/min window, 429 error responses]

## 6. Markdown Preservation
### Decision: [remark + unified AST parsing, preserve code/math/image nodes]
### Rationale: [Industry standard, preserves structure, Claude API respects delimiters]
### Alternatives Considered: [Regex (fragile), manual parsing (complex)]
### Implementation Notes: [Extract preserved nodes, send text-only to LLM, reassemble AST]
```

---

## Phase 1: Design & Contracts

**Prerequisites**: `research.md` complete

### 1. Data Model Design

**File**: `specs/009-auth-personalization/data-model.md`

**Entities**:

#### User (Managed by Better-Auth)
```typescript
interface User {
  id: string;              // UUID v4
  email: string;           // Unique, validated
  password_hash: string;   // bcrypt cost factor 12
  created_at: Date;
  last_login: Date | null;
}
```

#### Session (Managed by Better-Auth)
```typescript
interface Session {
  session_token: string;   // JWT signed with RS256
  user_id: string;         // FK to User.id
  expires_at: Date;        // 24h standard, 7d remember_me
  remember_me: boolean;
}
```

#### UserProfile (Custom Extension)
```typescript
interface UserProfile {
  user_id: string;                    // PK + FK to User.id
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
  language_preference: 'English' | 'Urdu';

  // EXISTING FIELDS (preserve from current schema)
  python_level: string;               // Keep existing
  ros2_level: string;                 // Keep existing
  gpu_available: string;              // Keep existing
  hardware_tier: string;              // Keep existing
  primary_goal: string;               // Keep existing

  created_at: Date;
  updated_at: Date;
}
```

#### TransformationCache (New)
```typescript
interface TransformationCache {
  id: string;                      // UUID v4
  cache_key: string;               // SHA-256 hash of (chapter_path + user_profile + transformation_type)
  transformed_content: string;     // Markdown output from LLM
  transformation_metadata: JSON;   // { changes_made: number, model: string, ... }
  created_at: Date;
  expires_at: Date;                // created_at + 5 minutes

  // Indexes
  INDEX idx_cache_key ON (cache_key);
  INDEX idx_expires_at ON (expires_at); // For cleanup jobs
}
```

**Relationships**:
- User 1:1 UserProfile (cascade delete)
- User 1:N Session (cascade delete)
- No foreign keys on TransformationCache (ephemeral data)

**Migrations**:
1. `001_better_auth_schema.sql`: Better-Auth auto-generated tables
2. `002_user_profiles_extension.sql`: Add new fields to existing user_profiles table
3. `003_transformation_cache.sql`: Create transformation_cache table with indexes

### 2. API Contracts

**Directory**: `specs/009-auth-personalization/contracts/`

#### File: `auth.openapi.yaml`

```yaml
openapi: 3.0.0
info:
  title: Authentication API
  version: 1.0.0

paths:
  /api/auth/signup:
    post:
      summary: Create new user account with profile
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
                password:
                  type: string
                  minLength: 8
                profile:
                  type: object
                  required: [software_background, hardware_experience]
                  properties:
                    software_background:
                      type: string
                      enum: [Beginner, Intermediate, Expert]
                    hardware_experience:
                      type: string
                      enum: [None, Basic, Advanced]
      responses:
        201:
          description: User created successfully
          content:
            application/json:
              schema:
                type: object
                properties:
                  token: { type: string }
                  user: { $ref: '#/components/schemas/User' }
                  profile: { $ref: '#/components/schemas/UserProfile' }
        400:
          description: Validation error
        409:
          description: Email already exists

  /api/auth/signin:
    post:
      summary: Authenticate existing user
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [email, password]
              properties:
                email: { type: string }
                password: { type: string }
                rememberMe: { type: boolean, default: false }
      responses:
        200:
          description: Login successful
          content:
            application/json:
              schema:
                type: object
                properties:
                  token: { type: string }
                  user: { $ref: '#/components/schemas/User' }
                  profile: { $ref: '#/components/schemas/UserProfile' }
        401:
          description: Invalid credentials

  /api/auth/profile:
    get:
      summary: Get current user profile
      security:
        - BearerAuth: []
      responses:
        200:
          description: Profile retrieved
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/UserProfile'
        401:
          description: Unauthorized

    put:
      summary: Update user profile
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/UserProfile'
      responses:
        200:
          description: Profile updated
        401:
          description: Unauthorized

components:
  securitySchemes:
    BearerAuth:
      type: http
      scheme: bearer
      bearerFormat: JWT

  schemas:
    User:
      type: object
      properties:
        id: { type: string, format: uuid }
        email: { type: string, format: email }
        created_at: { type: string, format: date-time }
        last_login: { type: string, format: date-time, nullable: true }

    UserProfile:
      type: object
      properties:
        software_background: { type: string, enum: [Beginner, Intermediate, Expert] }
        hardware_experience: { type: string, enum: [None, Basic, Advanced] }
        language_preference: { type: string, enum: [English, Urdu] }
        python_level: { type: string }
        ros2_level: { type: string }
```

#### File: `personalize.openapi.yaml`

```yaml
openapi: 3.0.0
info:
  title: Content Personalization API
  version: 1.0.0

paths:
  /api/personalize:
    post:
      summary: Personalize chapter content based on user profile
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [chapter_content]
              properties:
                chapter_content:
                  type: string
                  description: Markdown content of the chapter
                chapter_path:
                  type: string
                  description: URL path for cache key (e.g., /docs/module1/sensors)
      responses:
        200:
          description: Personalized content returned
          content:
            application/json:
              schema:
                type: object
                properties:
                  transformed_content:
                    type: string
                    description: Personalized markdown
                  transformation_metadata:
                    type: object
                    properties:
                      changes_made: { type: integer }
                      complexity_level: { type: string }
                      model: { type: string }
                      cached: { type: boolean }
        400:
          description: Invalid input
        401:
          description: Unauthorized
        429:
          description: Rate limit exceeded (10 req/user/min)
        503:
          description: AI service unavailable
```

#### File: `translate.openapi.yaml`

```yaml
openapi: 3.0.0
info:
  title: Urdu Translation API
  version: 1.0.0

paths:
  /api/translate:
    post:
      summary: Translate chapter content to Urdu
      security:
        - BearerAuth: []
      requestBody:
        required: true
        content:
          application/json:
            schema:
              type: object
              required: [chapter_content, target_language]
              properties:
                chapter_content: { type: string }
                target_language: { type: string, enum: [urdu] }
                chapter_path: { type: string }
      responses:
        200:
          description: Translated content returned
          content:
            application/json:
              schema:
                type: object
                properties:
                  transformed_content: { type: string }
                  transformation_metadata:
                    type: object
                    properties:
                      preserved_terms: { type: array, items: { type: string } }
                      model: { type: string }
                      cached: { type: boolean }
        429:
          description: Rate limit exceeded
```

### 3. Quickstart Guide

**File**: `specs/009-auth-personalization/quickstart.md`

```markdown
# Quickstart: Auth & Personalization Setup

## Prerequisites

- Node.js 20+
- Python 3.11+
- Neon Serverless Postgres account
- Anthropic API key

## Setup Steps

### 1. Database Setup

```bash
# Create Neon project at https://neon.tech
# Copy connection string to .env

# Backend .env
echo "DATABASE_URL=postgresql://user:pass@host/db?sslmode=require" > backend/.env
echo "ANTHROPIC_API_KEY=sk-ant-..." >> backend/.env
echo "JWT_SECRET=$(openssl rand -hex 32)" >> backend/.env

# Frontend .env.local
echo "REACT_APP_API_URL=http://localhost:4000" > textbook/.env.local
```

### 2. Install Dependencies

```bash
# Backend
cd backend
npm install

# Frontend
cd ../textbook
npm install

# Skills (Claude Agent SDK)
cd ../.claude/skills
# No installation needed - skills are markdown files
```

### 3. Run Migrations

```bash
cd backend
npm run migrate:up
```

### 4. Start Services

```bash
# Terminal 1: Backend
cd backend
npm run dev

# Terminal 2: Frontend
cd textbook
npm run start
```

### 5. Test Setup

```bash
# Create test user
curl -X POST http://localhost:4000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "testpass123",
    "profile": {
      "software_background": "Beginner",
      "hardware_experience": "None"
    }
  }'

# Navigate to http://localhost:3000/docs/intro
# Click "Personalize Chapter" button
# Verify content adapts to Beginner level
```

## Verification Checklist

- [ ] Database connection successful
- [ ] User signup works
- [ ] User login returns JWT token
- [ ] Profile update persists
- [ ] Personalize button visible to authenticated users
- [ ] Personalization completes in < 8s
- [ ] Translation preserves code blocks
- [ ] RAG chatbot still functional (untouched)
- [ ] Mobile responsive (44px touch targets)
```

### 4. Agent Context Update

Run the agent context update script:

```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook
.specify/scripts/bash/update-agent-context.sh claude
```

This will update `.claude/context.md` with new technologies:
- Better-Auth
- Neon Serverless Postgres
- Drizzle ORM
- Claude Agent SDK
- Express rate limiting

---

## Phase 2: Task Breakdown (Deferred to `/sp.tasks`)

**NOT CREATED BY THIS COMMAND**. Run `/sp.tasks` after approving this plan to generate:
- `specs/009-auth-personalization/tasks.md`
- Dependency-ordered implementation tasks
- Test acceptance criteria for each task

---

## Safety Guardrails: Existing RAG Chatbot Preservation

**Explicit Confirmation**: The following files and features MUST remain unchanged:

### Untouched Files
- `textbook/src/components/Chatbot/index.js` ✅
- `textbook/src/theme/Root.js` ✅
- `textbook/src/css/custom.css` (lines 32-513: chatbot styles) ✅

### Untouched Functionality
- RAG chatbot FAB button (z-index: 999999)
- Glassmorphism chat window
- Typewriter effect
- Citation badges
- Backend API: `https://khans-rag-project-rag-api.hf.space/chat`

### Z-Index Layer Separation
- **RAG Chatbot FAB**: `z-index: 999999` (highest)
- **Personalization Buttons**: `z-index: 1000` (below chatbot)
- **Chat Window**: `z-index: 999998` (below FAB)

### CSS Namespace Isolation
- Chatbot styles: `.chatbot-*` classes
- Personalization styles: `.personalize-*` classes
- No class name collisions

### Testing Verification
- **SC-008**: Zero interference between personalization UI and existing RAG chatbot
- Test: Click FAB while personalization buttons visible → chatbot opens
- Test: Personalize chapter with chatbot open → no z-index conflicts
- Test: Mobile viewport → both features accessible

---

## Deliverables Summary

**Phase 0 (Research)**:
- ✅ `research.md`: Technology validation and decision rationale

**Phase 1 (Design)**:
- ✅ `data-model.md`: Entity schemas, relationships, migrations
- ✅ `contracts/auth.openapi.yaml`: Authentication API contract
- ✅ `contracts/personalize.openapi.yaml`: Personalization API contract
- ✅ `contracts/translate.openapi.yaml`: Translation API contract
- ✅ `quickstart.md`: Setup guide with verification checklist
- ✅ Updated `.claude/context.md`: New technologies registered

**Next Steps**:
1. Review this plan for accuracy and completeness
2. Run `/sp.tasks` to generate implementation task breakdown
3. Begin implementation with Task 1 (highest priority)

---

**Plan Status**: ✅ READY FOR REVIEW
**Branch**: `009-auth-personalization`
**Date**: 2026-01-01
**Architect**: Claude Sonnet 4.5 (Spec-Kit Agent)
