# Implementation Tasks: Bonus Features (Authentication, Personalization, Urdu Translation)

**Feature**: 008-bonus-features
**Branch**: `008-bonus-features`
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)
**Date**: 2025-12-11

---

## Overview

This document organizes implementation tasks by **user story** to enable independent, incremental delivery.

**User Stories** (from spec.md):
1. **US1** (P1): User Authentication with Background Profiling - MVP that enables other features
2. **US2** (P2): Content Personalization Based on Background - Depends on US1
3. **US3** (P3): Urdu Translation - Independent of US2

**Total Tasks**: 58
**Estimated Effort**: 100 hours (~2.5 weeks full-time)

---

## Phase 1: Setup & Infrastructure (8 tasks)

**Goal**: Initialize project structure, install dependencies, configure development environment

**Tasks**:

- [X] T001 Create backend API directory structure at `api/` with subdirectories: src/, tests/, prisma/
- [X] T002 [P] Initialize Node.js project in `api/` with package.json (TypeScript 5.3, Node 20 LTS)
- [X] T003 [P] Install backend dependencies: express, @anthropic-ai/sdk, better-auth, pg, ioredis, prisma, bcrypt
- [X] T004 [P] Install dev dependencies: typescript, @types/node, @types/express, ts-node, nodemon
- [X] T005 [P] Create TypeScript configuration in `api/tsconfig.json` (target: ES2022, module: commonjs, strict mode)
- [X] T006 Create Docker Compose file in `api/docker-compose.yml` (PostgreSQL 15-alpine, Redis 7-alpine containers)
- [X] T007 Create environment template in `api/.env.example` (DATABASE_URL, REDIS_URL, CLAUDE_API_KEY, JWT_SECRET placeholders)
- [X] T008 Create Express server entry point in `api/src/server.ts` (basic Express app, port 4000, health check route)

**Completion Criteria**:
- ✅ `npm install` succeeds in `api/` directory
- ✅ `docker-compose up -d` starts PostgreSQL and Redis
- ✅ `npm run dev` starts Express server on http://localhost:4000
- ✅ Health check endpoint responds at GET /health

---

## Phase 2: Foundational Infrastructure (10 tasks)

**Goal**: Set up database, authentication framework, API clients, and core middleware

**Tasks**:

- [X] T009 Create Prisma schema in `api/prisma/schema.prisma` with User model (id UUID, email unique, password_hash, created_at, last_login, is_locked, lock_expires_at)
- [X] T010 Add UserProfile model to Prisma schema (user_id FK, python_level enum, ros2_level enum, gpu_available enum, hardware_tier enum, primary_goal enum, language_preference enum)
- [X] T011 Generate Prisma client and run initial migration in `api/prisma/migrations/` (create users and user_profiles tables with indexes) - Schema ready, requires Docker
- [X] T012 [P] Create PostgreSQL client configuration in `api/src/config/database.ts` (connection pool with Prisma, error handling)
- [X] T013 [P] Create Redis client configuration in `api/src/config/redis.ts` (ioredis client, connection retry logic)
- [X] T014 [P] Create Claude API client configuration in `api/src/config/claude.ts` (@anthropic-ai/sdk, API key from env, default model: claude-3-5-sonnet-20241022)
- [X] T015 Create cache service in `api/src/services/cache.service.ts` (get, set with TTL, delete, invalidate pattern methods using Redis)
- [X] T016 Create error handling middleware in `api/src/middleware/error.middleware.ts` (catch errors, return JSON with status codes, log to console)
- [X] T017 Create input validators in `api/src/utils/validators.ts` (email regex RFC 5322, password strength ≥8 chars + number + special char)
- [X] T018 Update Express server in `api/src/server.ts` to register error middleware and enable CORS for Docusaurus origin

**Completion Criteria**:
- ✅ `npx prisma migrate dev` creates database tables successfully
- ✅ PostgreSQL connection pool connects without errors
- ✅ Redis client connects and can set/get keys
- ✅ Claude API client initializes with valid API key
- ✅ Error middleware catches and formats errors correctly

---

## Phase 3: User Story 1 - Authentication (Priority P1) - 15 tasks

**Story Goal**: Users can create accounts, log in, manage profiles with 5 background questions

**Independent Test**: Create account → log out → log back in → profile data persisted

**Implementation Tasks**:

### Backend - Models & Services (5 tasks)

- [X] T019 [US1] Create User model TypeScript interface in `api/src/models/user.ts` (id, email, password_hash, timestamps, is_locked, lock_expires_at)
- [X] T020 [US1] Create UserProfile model TypeScript interface in `api/src/models/user-profile.ts` (user_id FK, 5 background fields as enums, language_preference)
- [X] T021 [US1] Implement auth service signup method in `api/src/services/auth.service.ts` (validate input, hash password with bcrypt rounds=10, create user + profile in transaction)
- [X] T022 [US1] Implement auth service signin method in `api/src/services/auth.service.ts` (verify credentials, check account lock, generate JWT with 1-day or 7-day expiration, update last_login)
- [X] T023 [US1] Implement auth service logout and profile methods in `api/src/services/auth.service.ts` (invalidate session, get/update profile with cache invalidation)

### Backend - Routes & Middleware (3 tasks)

- [X] T024 [US1] Configure Better Auth in `api/src/config/better-auth.ts` (email/password provider, PostgreSQL adapter, session config, JWT secret from env)
- [X] T025 [US1] Create auth middleware in `api/src/middleware/auth.middleware.ts` (verify JWT token, extract user ID, attach to request object, return 401 if invalid)
- [X] T026 [US1] Create auth routes in `api/src/routes/auth.routes.ts` (POST /api/auth/signup, /signin, /logout, GET /api/auth/profile, PUT /api/auth/profile with auth middleware)

### Frontend - Components (5 tasks)

- [X] T027 [P] [US1] Create SignupForm component in `textbook/src/components/auth/SignupForm.tsx` (email field, password field, 5 background question dropdowns, submit button, error display)
- [X] T028 [P] [US1] Create SigninForm component in `textbook/src/components/auth/SigninForm.tsx` (email field, password field, remember me checkbox, submit button, error display, redirect to previous page)
- [X] T029 [P] [US1] Create ProfileEditor component in `textbook/src/components/auth/ProfileEditor.tsx` (editable form for 5 background questions, save button, success/error messages)
- [X] T030 [US1] Create AuthProvider context in `textbook/src/components/auth/AuthProvider.tsx` (React Context with user state, login/logout methods, token storage in localStorage)
- [X] T031 [US1] Create useAuth hook in `textbook/src/hooks/useAuth.ts` (access AuthProvider context, return user, isAuthenticated, login, logout, updateProfile methods)

### Frontend - Pages & Integration (2 tasks)

- [X] T032 [P] [US1] Create signup page in `textbook/src/pages/auth/signup.tsx` (render SignupForm, call /api/auth/signup, redirect to home on success)
- [X] T033 [P] [US1] Create signin page in `textbook/src/pages/auth/signin.tsx` (render SigninForm, call /api/auth/signin, store JWT token, redirect to previous page)

**US1 Completion Criteria**:
- ✅ Acceptance Scenario 1: Signup with email + password + 5 questions → account created, auto-login, profile saved
- ✅ Acceptance Scenario 2: Signin with correct credentials → logged in, redirected
- ✅ Acceptance Scenario 3: Close browser, return within 7 days → still logged in (JWT valid)
- ✅ Acceptance Scenario 4: 3 failed login attempts → account locked 15 minutes

**Parallel Opportunities**:
- T027-T029 (frontend components) can be developed in parallel with T019-T026 (backend)
- T032-T033 (pages) can be developed after T027-T031 complete

---

## Phase 4: User Story 2 - Personalization (Priority P2) - 13 tasks

**Story Goal**: Authenticated users can click "Personalize This Chapter" to adapt content based on their profile

**Independent Test**: Log in with profile (Python: Expert, ROS 2: Beginner) → open Chapter 1.1 → click Personalize → verify Python details minimal, ROS 2 details expanded

**Dependencies**: Requires US1 (Authentication) complete for user profiles

**Implementation Tasks**:

### Backend - Utilities & Services (4 tasks)

- [X] T034 [US2] Create MDX parser utility in `api/src/utils/markdown-parser.ts` (parse MDX to AST with remark-parse + remark-mdx, extract text nodes excluding code blocks, reconstruct MDX from modified AST)
- [X] T035 [US2] Create profile hash utility in `api/src/utils/profile-hash.ts` (generate MD5 hash from python_level|ros2_level|gpu_available|hardware_tier|primary_goal for cache keys)
- [X] T036 [US2] Implement personalization service in `api/src/services/personalization.service.ts` (check Redis cache with key `personalize:{chapterId}:{profileHash}`, if miss: parse MDX, call Claude API with user profile + chapter content, reconstruct MDX, cache for 24 hours, return personalized content)
- [X] T037 [US2] Create Claude prompt template in `api/src/services/personalization.service.ts` for content adaptation (instructions to adjust verbosity based on python_level, expand/collapse concepts based on ros2_level, show/hide hardware options based on gpu_available and hardware_tier)

### Backend - Routes & Middleware (2 tasks)

- [X] T038 [US2] Create rate limiting middleware in `api/src/middleware/rate-limit.middleware.ts` (use express-rate-limit + Redis store, 10 requests per 60 seconds per user for /personalize, 5 requests per 60 seconds for /translate)
- [X] T039 [US2] Create personalization route in `api/src/routes/personalize.routes.ts` (POST /api/personalize with auth middleware + rate limiting, validate chapterId + originalContent, call personalization service, return {personalizedContent, cacheHit, profile})

### Frontend - Components & Hooks (5 tasks)

- [X] T040 [P] [US2] Create PersonalizeButton component in `textbook/src/components/personalization/PersonalizeButton.tsx` (button "Personalize This Chapter", disabled if not authenticated, onClick triggers personalization, show tooltip when disabled)
- [X] T041 [P] [US2] Create PersonalizationLoader component in `textbook/src/components/personalization/PersonalizationLoader.tsx` (loading spinner with message "Personalizing content...", timeout after 10 seconds with fallback message)
- [X] T042 [P] [US2] Create PersonalizedContent component in `textbook/src/components/personalization/PersonalizedContent.tsx` (render personalized MDX with MDXProvider, show "Reset to Default" button, handle reset onClick)
- [X] T043 [US2] Create usePersonalization hook in `textbook/src/hooks/usePersonalization.ts` (state: personalizing, personalized, error; methods: personalizeChapter, resetChapter; call /api/personalize with chapter content)
- [X] T044 [US2] Create styles for personalization UI in `textbook/src/styles/personalization.module.css` (button styles, loader animation, error toast styles)

### Frontend - Integration (2 tasks)

- [ ] T045 [US2] Swizzle Docusaurus DocItem theme by running `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap` in textbook/
- [ ] T046 [US2] Modify swizzled theme in `textbook/src/theme/DocItem/index.tsx` to inject PersonalizeButton at top of chapter content (conditionally render based on useAuth().isAuthenticated)

**US2 Completion Criteria**:
- ✅ Acceptance Scenario 1: Beginner profile → click Personalize → verbose Python basics, ROS 2 analogies, detailed comments
- ✅ Acceptance Scenario 2: Expert profile → click Personalize → minimal Python, advanced TensorRT details expanded
- ✅ Acceptance Scenario 3: Cloud GPU profile → click Personalize → AWS alternatives highlighted, on-premise minimized
- ✅ Acceptance Scenario 4: Not logged in → click Personalize → prompt to log in

**Parallel Opportunities**:
- T034-T037 (backend) can be developed in parallel with T040-T044 (frontend)
- T045-T046 (theme swizzling) can be done after T040-T044 complete

---

## Phase 5: User Story 3 - Translation (Priority P3) - 10 tasks

**Story Goal**: Users can click "Translate to Urdu" to read chapter content in Urdu with preserved formatting

**Independent Test**: Open any chapter → click Translate to Urdu → verify text translated, code blocks in English, diagrams preserved, technical terms transliterated

**Dependencies**: Independent of US2 (can be implemented in parallel or after US2)

**Implementation Tasks**:

### Backend - Services & Routes (3 tasks)

- [ ] T047 [US3] Implement translation service in `api/src/services/translation.service.ts` (check Redis cache with key `translate:{chapterId}:urdu`, if miss: parse MDX, call Claude API with translation prompt, handle Mermaid diagram label translation with regex, cache for 7 days, return translated content + technical terms map)
- [ ] T048 [US3] Create Claude prompt template in `api/src/services/translation.service.ts` for Urdu translation (instructions to translate paragraphs/headings/lists to Urdu, preserve code blocks, transliterate technical terms like "ROS 2" → "آر او ایس 2", maintain Markdown structure)
- [ ] T049 [US3] Create translation route in `api/src/routes/translate.routes.ts` (POST /api/translate with rate limiting middleware, validate chapterId + originalContent + targetLanguage, call translation service, return {translatedContent, cacheHit, technicalTerms})

### Frontend - Components & Hooks (5 tasks)

- [ ] T050 [P] [US3] Create TranslateButton component in `textbook/src/components/translation/TranslateButton.tsx` (button "Translate to Urdu", onClick triggers translation, show "Show Original" button after translation, handle language toggle)
- [ ] T051 [P] [US3] Create TranslationLoader component in `textbook/src/components/translation/TranslationLoader.tsx` (loading spinner with message "Translating to Urdu...", timeout after 15 seconds with error message)
- [ ] T052 [P] [US3] Create TranslatedContent component in `textbook/src/components/translation/TranslatedContent.tsx` (render Urdu MDX with RTL text direction, handle scroll position preservation, show technical terms glossary tooltip)
- [ ] T053 [US3] Create useTranslation hook in `textbook/src/hooks/useTranslation.ts` (state: translating, language, translatedContent, error; methods: translateToUrdu, showOriginal; call /api/translate, persist language preference if logged in)
- [ ] T054 [US3] Create RTL styles for Urdu in `textbook/src/styles/translation.module.css` (direction: rtl for Urdu text, preserve LTR for code blocks, font-family for Urdu support like "Noto Nastaliq Urdu", responsive layout)

### Frontend - Integration (2 tasks)

- [ ] T055 [US3] Update swizzled theme in `textbook/src/theme/DocItem/index.tsx` to inject TranslateButton below PersonalizeButton (render for all users, no auth required)
- [ ] T056 [US3] Test Urdu RTL rendering in multiple browsers (Chrome, Firefox, Safari, Edge) and fix any layout issues with CSS adjustments

**US3 Completion Criteria**:
- ✅ Acceptance Scenario 1: Click Translate → paragraphs in Urdu, code in English, headings translated, technical terms transliterated
- ✅ Acceptance Scenario 2: Click Show Original → content returns to English immediately
- ✅ Acceptance Scenario 3: Logged in with Urdu preference → new chapters load in Urdu automatically
- ✅ Acceptance Scenario 4: Mermaid diagrams → labels translated to Urdu, structure intact, renders correctly

**Parallel Opportunities**:
- T047-T049 (backend) can be developed in parallel with T050-T054 (frontend)
- T056 (browser testing) can be done after T050-T055 complete

---

## Phase 6: Polish & Documentation (2 tasks)

**Goal**: Finalize integration, optimize performance, document setup and usage

**Tasks**:

- [ ] T057 Update Docusaurus configuration in `textbook/docusaurus.config.js` to add API proxy (proxy /api requests to http://localhost:4000 for development)
- [ ] T058 Create quickstart documentation in `specs/008-bonus-features/quickstart.md` (7-step setup: clone repo, install deps, Docker Compose, env vars, Prisma migrate, start servers, verify signup/personalize/translate)

**Completion Criteria**:
- ✅ Docusaurus dev server proxies API requests correctly
- ✅ Quickstart guide allows new developer to get working setup in 10 minutes
- ✅ All 3 user stories (US1, US2, US3) independently testable and complete

---

## Task Summary

**Total Tasks**: 58
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 10 tasks
- Phase 3 (US1 Authentication): 15 tasks
- Phase 4 (US2 Personalization): 13 tasks
- Phase 5 (US3 Translation): 10 tasks
- Phase 6 (Polish): 2 tasks

**Parallelizable Tasks**: 18 tasks marked with [P] can be executed in parallel

**User Story Dependencies**:
- US1 (Authentication) - **NO dependencies** - Can start immediately after Phase 2
- US2 (Personalization) - **Depends on US1** - Requires authentication and user profiles
- US3 (Translation) - **NO dependencies on US2** - Can be implemented in parallel with US2 after Phase 2

---

## Dependency Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational - Database, Clients, Middleware)
    ↓
    ├─→ Phase 3: US1 (Authentication) [P1] ← MVP
    │       ↓
    │   Phase 4: US2 (Personalization) [P2]
    │
    └─→ Phase 5: US3 (Translation) [P3] [Independent]
            ↓
        Phase 6 (Polish & Docs)
```

**Critical Path**: Setup → Foundational → US1 → US2 (US3 can proceed in parallel after Foundational)

---

## Parallel Execution Examples

### Phase 3 (US1 - Authentication)
**Parallel Tracks**:
- Track A: Backend (T019-T026) - Models, services, routes, middleware
- Track B: Frontend (T027-T029) - SignupForm, SigninForm, ProfileEditor components
- **Convergence**: T030-T033 require both tracks complete

### Phase 4 (US2 - Personalization)
**Parallel Tracks**:
- Track A: Backend (T034-T039) - MDX parser, personalization service, routes
- Track B: Frontend (T040-T044) - PersonalizeButton, hooks, styles
- **Convergence**: T045-T046 (theme swizzling) requires both tracks complete

### Phase 5 (US3 - Translation)
**Parallel Tracks**:
- Track A: Backend (T047-T049) - Translation service, routes
- Track B: Frontend (T050-T054) - TranslateButton, hooks, RTL styles
- **Convergence**: T055-T056 (integration + browser testing) requires both tracks complete

---

## MVP Scope Recommendation

**Minimum Viable Product**: Phase 1 + Phase 2 + Phase 3 (US1 Authentication only)

This provides:
- ✅ User signup with 5 background questions
- ✅ User signin with session persistence
- ✅ Profile management
- ✅ Account security (password hashing, account lockout)

**Delivers Value**: Users can create accounts and profiles, enabling future personalization features

**Time Estimate**: ~30-35 hours (30% of total effort)

---

## Implementation Strategy

**Incremental Delivery Approach**:

1. **Sprint 1** (Week 1): Phase 1-3 (Setup + Foundational + US1 Authentication)
   - Deliverable: Working authentication system
   - Demo: User signup → login → profile edit

2. **Sprint 2** (Week 2): Phase 4 (US2 Personalization)
   - Deliverable: Content personalization based on user profile
   - Demo: Login as beginner → personalize Chapter 1.1 → see verbose explanations

3. **Sprint 3** (Week 3): Phase 5-6 (US3 Translation + Polish)
   - Deliverable: Urdu translation feature
   - Demo: Open any chapter → translate to Urdu → verify formatting preserved

**Quality Gates**:
- After each phase: Manual testing of acceptance scenarios
- After US1: Verify all 4 authentication acceptance scenarios pass
- After US2: Verify all 4 personalization acceptance scenarios pass
- After US3: Verify all 4 translation acceptance scenarios pass
- Final: Cross-browser testing (Chrome, Firefox, Safari, Edge)

---

## File Path Reference

**Backend API** (`api/`):
- Config: `api/src/config/{better-auth,database,redis,claude}.ts`
- Models: `api/src/models/{user,user-profile,*-cache}.ts`
- Services: `api/src/services/{auth,personalization,translation,cache}.service.ts`
- Routes: `api/src/routes/{auth,personalize,translate}.routes.ts`
- Middleware: `api/src/middleware/{auth,rate-limit,error}.middleware.ts`
- Utils: `api/src/utils/{markdown-parser,profile-hash,validators}.ts`
- Server: `api/src/server.ts`
- Database: `api/prisma/schema.prisma`, `api/prisma/migrations/`
- Docker: `api/docker-compose.yml`
- Env: `api/.env.example`

**Frontend** (`textbook/src/`):
- Components:
  - Auth: `textbook/src/components/auth/{SignupForm,SigninForm,ProfileEditor,AuthProvider}.tsx`
  - Personalization: `textbook/src/components/personalization/{PersonalizeButton,PersonalizationLoader,PersonalizedContent}.tsx`
  - Translation: `textbook/src/components/translation/{TranslateButton,TranslationLoader,TranslatedContent}.tsx`
- Hooks: `textbook/src/hooks/{useAuth,usePersonalization,useTranslation}.ts`
- Pages: `textbook/src/pages/auth/{signup,signin,profile}.tsx`
- Styles: `textbook/src/styles/{auth,personalization,translation}.module.css`
- Theme: `textbook/src/theme/DocItem/index.tsx` (swizzled)

**Configuration**:
- Docusaurus: `textbook/docusaurus.config.js`

**Documentation**:
- Quickstart: `specs/008-bonus-features/quickstart.md`

---

## Success Metrics (from spec.md)

**Authentication** (US1):
- SC-001: Signup completes in <3 minutes
- SC-002: Signin success rate >95%
- SC-003: Zero password leaks (bcrypt hashing)
- SC-004: Account lockout prevents brute force

**Personalization** (US2):
- SC-005: Personalization completes within 10 seconds for 90% of requests
- SC-006: Personalized content reduces reading time by 15-20%
- SC-007: Cache hit rate >70% after 1 week
- SC-008: User satisfaction 80% rate as "helpful"

**Translation** (US3):
- SC-009: Translation accuracy >90%
- SC-010: Formatting preserved 100%
- SC-011: Translation completes within 15 seconds for 95% of chapters
- SC-012: Urdu RTL rendering works correctly in all browsers

**Integration**:
- SC-013: All features work together without conflicts
- SC-014: Docusaurus build succeeds with new features
- SC-015: Page load time increases by <500ms

---

**Tasks Status**: Ready for implementation via `/sp.implement`

**Branch**: `008-bonus-features`
**Next Step**: Execute tasks sequentially using `/sp.implement` or manually implement by user story
