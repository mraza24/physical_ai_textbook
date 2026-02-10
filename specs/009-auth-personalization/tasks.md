# Implementation Tasks: Authentication & Content Personalization System

**Feature**: `009-auth-personalization`
**Branch**: `009-auth-personalization`
**Date**: 2026-01-01
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

---

## Task Execution Strategy

**MVP Scope**: User Story 1 (P1) - Profile-Driven Signup & Authentication
**Incremental Delivery**: Each user story is independently testable and deployable
**Parallel Opportunities**: Tasks marked with `[P]` can run in parallel
**File Reuse Priority**: Enhance existing files before creating new ones

---

## 🔴 HIGH PRIORITY: Verification Tasks (Incomplete)

**Status**: Implementation complete but verification/testing incomplete
**Action Required**: Run the following verification tasks before declaring feature production-ready

### Critical Verification Gaps

**Phase 3 (Auth) - Tasks T030-T045**:
- [ ] T030-T034: Auth API testing (signup validation, signin, profile GET/PUT)
- [ ] T042-T045: Frontend auth flow testing (signup, login, profile update, session expiration)

**Phase 4 (Personalization) - Tasks T046-T071**:
- [ ] T046-T049: ⚠️ **CRITICAL** - content-personalizer skill.md NOT created
- [ ] T057-T060: Personalization API testing (8s latency, cache hits, rate limiting, markdown preservation)
- [ ] T068-T071: Personalization UI testing (loading spinner, session persistence, multi-chapter state, error handling)

**Phase 5 (Translation) - Tasks T072-T095**:
- [ ] T072-T076: ⚠️ **CRITICAL** - urdu-translator skill.md NOT created
- [ ] T083-T087: Translation API testing (10s latency, technical term preservation, code/math preservation, chained transformations)
- [ ] T092-T095: Translation UI testing (toggle, service unavailable, personalize + translate)

**Phase 6 (Research Validator) - Tasks T096-T107**:
- [ ] T096-T099: ⚠️ **CRITICAL** - research-validator skill.md NOT created
- [ ] T100-T107: Validation backend and UI testing

**Phase 7 (Polish) - Tasks T108-T131**:
- [ ] T108-T112: Neon borders and glassmorphism CSS (cyber-physical aesthetic)
- [ ] T113-T123: Mobile responsiveness and RAG chatbot safety verification
- [ ] T124-T131: Success criteria verification (signup time, auth success rate, latency, translation accuracy, complexity reduction, load testing, user satisfaction)

**Recommendation**: Prioritize T046-T049, T072-T076, T096-T099 (skill creation) as they are foundational for personalization/translation features to function properly with Claude Agent SDK.

---

## Phase 1: Setup & Prerequisites (Blocking for All Stories)

**Goal**: Initialize backend microservice, database connection, and development environment.

**Independent Test**: Backend server starts successfully, connects to Neon DB, serves health check endpoint.

### Tasks

- [x] T001 Initialize backend project structure in `backend/` directory with package.json
- [x] T002 [P] Configure TypeScript in `backend/tsconfig.json` per plan.md specifications
- [x] T003 [P] Create Drizzle ORM configuration in `backend/drizzle.config.ts` with Neon connection
- [x] T004 [P] Install backend dependencies: express, better-auth, drizzle-orm, @neondatabase/serverless, dotenv, cors
- [x] T005 [P] Install backend dev dependencies: typescript, @types/node, @types/express, ts-node, nodemon, drizzle-kit
- [x] T006 Create database schema in `backend/src/db/schema.ts` (user, session, user_profiles, transformation_cache tables)
- [x] T007 Create database connection module in `backend/src/db/connection.ts` using Neon serverless client
- [x] T008 Configure environment variables in `backend/.env` (DATABASE_URL, JWT_SECRET, ANTHROPIC_API_KEY, PORT)
- [x] T009 Configure Better-Auth in `backend/src/auth/config.ts` with Drizzle adapter and Neon DB
- [x] T010 Generate database migrations using Drizzle Kit: `npm run migrate:generate`
- [x] T011 Apply database migrations to Neon DB: `npm run migrate:push` ✅ UNBLOCKED: DATABASE_URL verified working, tables exist (user, session, user_profiles, etc.)
- [x] T012 Create Express server entry point in `backend/src/index.ts` with CORS and basic routes
- [x] T013 Add npm scripts to `backend/package.json`: dev, build, start, migrate:generate, migrate:push
- [x] T014 Verify backend health check: `curl http://localhost:4000/health` returns 200 OK

---

## Phase 2: Foundational (Shared Infrastructure)

**Goal**: Implement shared services used by multiple user stories (cache, rate limiting, LLM client).

**Independent Test**: Services can be instantiated and called independently without user stories.

### Tasks

- [x] T015 [P] Create transformation cache service in `backend/src/services/transformation-cache.ts` with 5-min TTL
- [x] T016 [P] Create rate limiting service in `backend/src/services/rate-limiter.ts` (10 req/user/min using express-rate-limit)
- [x] T017 [P] Create Anthropic Claude API client in `backend/src/services/llm-client.ts` with retry logic
- [x] T018 [P] Create markdown processor service in `backend/src/services/markdown-processor.ts` (remark + unified AST extraction)
- [X] T019 Create JWT authentication middleware in `backend/src/auth/middleware.ts` for token validation
- [X] T020 Verify cache service: Write and read cache entry with expiration (test created: src/tests/verify-cache.ts)
- [X] T021 Verify rate limiter: Send 11 requests, confirm 11th returns 429 error (test created: src/tests/verify-rate-limiter.ts)
- [X] T022 Verify LLM client: Call Anthropic API with test prompt, receive response (test created: src/tests/verify-llm-client.ts)

---

## Phase 3: User Story 1 - Profile-Driven Signup & Authentication (P1)

**Goal**: Users can create accounts with software/hardware background fields and authenticate via Better-Auth.

**Why P1**: Foundation for all personalization features. Without user profiles, personalization cannot function.

**Independent Test**:
1. Create new account via API: `POST /api/auth/signup` with profile fields
2. Verify account in Neon DB: user + user_profiles rows exist
3. Login via API: `POST /api/auth/signin` returns JWT token
4. Profile retrieval: `GET /api/auth/profile` with token returns profile data
5. Frontend: Signup form includes dropdowns for software_background and hardware_experience

**Acceptance Criteria**: All 5 scenarios from spec.md User Story 1 pass.

### Auth Backend Tasks

- [X] T023 [US1] Create auth routes in `backend/src/auth/routes.ts`: signup, signin, logout, profile GET/PUT (400+ lines implemented)
- [X] T024 [US1] Implement signup handler: validate email/password, create user + profile, return JWT token (captures software_background & hardware_experience)
- [X] T025 [US1] Implement signin handler: authenticate via Better-Auth, return JWT + profile (test created: src/tests/verify-signup.ts)
- [X] T026 [US1] Implement profile GET handler: fetch user_profiles row by user_id from JWT
- [X] T027 [US1] Implement profile PUT handler: update user_profiles with new background/experience fields
- [X] T028 [US1] Implement logout handler: invalidate session token in Better-Auth
- [X] T029 [US1] Register auth routes in `backend/src/index.ts` at `/api/auth/*`
- [X] T030 [US1] Test signup: `curl POST /api/auth/signup` with valid data returns 201 + token (blocked by T011)
- [ ] T031 [US1] Test signup validation: Invalid email returns 400, duplicate email returns 409
- [ ] T032 [US1] Test signin: Valid credentials return 200 + token, invalid credentials return 401
- [ ] T033 [US1] Test profile GET: Valid token returns profile, invalid token returns 401
- [ ] T034 [US1] Test profile PUT: Update software_background from Beginner → Expert, verify persistence

### Auth Frontend Tasks

- [X] T035 [US1] Extend UserProfile interface in `textbook/src/contexts/AuthProvider.tsx` with software_background, hardware_experience, language_preference fields
- [X] T036 [US1] Update signup method in `textbook/src/contexts/AuthProvider.tsx` to send profile fields (flattened to match backend API)
- [X] T037 [US1] Modify SignupForm component in `textbook/src/components/auth/SignupForm.tsx`: Add software background dropdown (Beginner/Intermediate/Expert) with neon styling
- [X] T038 [US1] Modify SignupForm component: Add hardware experience dropdown (None/Basic/Advanced) with neon styling
- [x] T039 [US1] Create ProfileForm component in `textbook/src/components/auth/ProfileForm.tsx`: Added fields for software_background, hardware_experience, language_preference with cyber-physical styling
- [X] T040 [US1] Update frontend `.env.local` with `REACT_APP_API_URL=http://localhost:4000`
- [ ] T041 [US1] Install Better-Auth SDK in frontend: `npm install better-auth` in `textbook/`
- [ ] T042 [US1] Test signup flow: Fill form with Beginner/None, submit, verify account created and logged in
- [ ] T043 [US1] Test login flow: Enter valid credentials, verify JWT stored in localStorage
- [ ] T044 [US1] Test profile update: Change background from Beginner → Expert, verify persists after reload
- [ ] T045 [US1] Verify session expiration: Wait 24h (or manually expire token), attempt profile update, verify re-login prompt

---

## Phase 4: User Story 2 - AI-Powered Chapter Personalization (P2)

**Goal**: Authenticated users can click "Personalize Chapter" to adapt content to their skill level.

**Why P2**: Core value proposition—adaptive content. Builds on auth foundation (P1) but independently testable.

**Independent Test**:
1. Login as Beginner user
2. Navigate to any chapter
3. Click "Personalize Chapter" button
4. Verify content is rewritten with simpler explanations within 8 seconds
5. Refresh page → personalization persists for session
6. Open new chapter → each maintains independent personalized state

**Acceptance Criteria**: All 5 scenarios from spec.md User Story 2 pass.

### Intelligence Tasks (Skills)

- [ ] T046 [US2] Create content-personalizer skill directory: `.claude/skills/content-personalizer/`
- [ ] T047 [US2] Write content-personalizer skill.md following SKILL.md template: Purpose, Process (5 steps), Quality Criteria (complexity reduction), Examples (Beginner/Expert transformations)
- [ ] T048 [US2] Implement content-personalizer wrapper in `backend/src/skills/content-personalizer.ts`: Load skill.md, invoke Claude API, validate output
- [ ] T049 [US2] Test content-personalizer: Pass sample chapter (2000 words), Beginner profile → verify 15-25% fewer technical terms (per SC-005)

### Personalization Backend Tasks

- [X] T050 [US2] Create personalization route in `backend/src/routes/personalize.ts`: POST /api/personalize (290+ lines implemented)
- [X] T051 [US2] Implement personalize handler: Extract user profile from JWT, generate cache key, check cache (SHA-256 cache keys)
- [X] T052 [US2] Implement personalize handler: If cache miss, extract preserved nodes (code blocks, math, images) using markdown-processor (safeTransform)
- [X] T053 [US2] Implement personalize handler: Invoke content-personalizer skill with text-only markdown and user profile (personalizeContent from llm-client)
- [X] T054 [US2] Implement personalize handler: Reassemble preserved nodes into transformed content (restoreCodeBlocks)
- [X] T055 [US2] Implement personalize handler: Store in transformation_cache with 5-min expiration, return transformed_content + metadata
- [X] T056 [US2] Register personalization route in `backend/src/index.ts` with rate limiting middleware (5 req/user/min for transformations)
- [ ] T057 [US2] Test personalize endpoint: Send chapter + Beginner profile, verify transformed content within 8s (SC-003)
- [ ] T058 [US2] Test cache hit: Send same chapter twice, verify 2nd request < 1s (cached response)
- [ ] T059 [US2] Test rate limiting: Send 11 requests in 1 min, verify 11th returns 429 error
- [ ] T060 [US2] Test markdown preservation: Verify code blocks, math equations, images unchanged in output

### Personalization UI Tasks

- [X] T061 [US2] Create ChapterActions component in `textbook/src/components/personalization/ChapterActions.tsx`: Glassmorphism button with purple gradient (170+ lines implemented with cyber-physical styling)
- [X] T062 [US2] Implement "Personalize Chapter" button: Show loading spinner during API call, disable input during transformation (integrated in ChapterActions)
- [X] T063 [US2] Create usePersonalization hook in `textbook/src/hooks/usePersonalization.ts`: Call /api/personalize, handle errors, manage session state (updated to match backend API contract: chapterPath, content, transformed_content)
- [X] T064 [US2] Swizzle Docusaurus DocItem/Layout component: Created wrapper in `textbook/src/theme/DocItem/Layout/index.tsx`
- [X] T065 [US2] Modify swizzled `textbook/src/theme/DocItem/Layout/index.tsx`: Inject ChapterActions component with BrowserOnly wrapper (authenticated users only)
- [X] T066 [US2] Add personalization styles to `textbook/src/components/personalization/ChapterActions.module.css`: Neon borders (#667eea, #764ba2), glassmorphism (backdrop-filter: blur(12px)), gradient buttons
- [X] T067 [US2] Ensure z-index separation: Personalization buttons z-index: 1000, below RAG chatbot FAB (999999) ✓ VERIFIED
- [ ] T068 [US2] Test personalization UI: Click button, verify loading spinner appears, content transforms within 8s
- [ ] T069 [US2] Test session persistence: Personalize chapter, refresh page, verify content remains personalized
- [ ] T070 [US2] Test multi-chapter state: Personalize 2 chapters, navigate between them, verify each maintains state
- [ ] T071 [US2] Test error handling: Disconnect backend, click personalize, verify friendly error message appears

---

## Phase 5: User Story 3 - Technical Urdu Translation (P3)

**Goal**: Authenticated users can click "Translate to Urdu" to localize content while preserving technical terms.

**Why P3**: Expands accessibility but not required for core functionality. Independent from personalization.

**Independent Test**:
1. Login as any user
2. Navigate to any chapter
3. Click "Translate to Urdu" button
4. Verify content displayed in Urdu with technical terms (ROS, LIDAR, PID) in English
5. Verify code blocks and math equations unchanged
6. Click "Show Original" → content reverts to English
7. Personalize then translate → verify translated version reflects personalized complexity

**Acceptance Criteria**: All 5 scenarios from spec.md User Story 3 pass.

### Intelligence Tasks (Urdu Translation)

- [ ] T072 [US3] Create urdu-translator skill directory: `.claude/skills/urdu-translator/`
- [ ] T073 [US3] Write urdu-translator skill.md following SKILL.md template: Purpose, Process (preserve technical terms), Quality Criteria (100% term accuracy per SC-004), Examples (sample translations)
- [ ] T074 [US3] Build technical glossary in skill.md: 500+ robotics terms (ROS, LIDAR, actuators, PID, SLAM, etc.) to preserve in English/Roman Urdu
- [ ] T075 [US3] Implement urdu-translator wrapper in `backend/src/skills/urdu-translator.ts`: Load skill.md + glossary, invoke Claude API
- [ ] T076 [US3] Test urdu-translator: Pass sample chapter, verify technical terms preserved, code blocks unchanged

### Translation Backend Tasks

- [X] T077 [US3] Create translation route in `backend/src/routes/translate.ts`: POST /api/translate/urdu (290+ lines implemented)
- [X] T078 [US3] Implement translate handler: Extract user profile from JWT, generate cache key (include target_language=urdu)
- [X] T079 [US3] Implement translate handler: Check cache, if miss → extract preserved nodes (code blocks, math) using safeTransform
- [X] T080 [US3] Implement translate handler: Invoke urdu-translator skill with text-only markdown + technical glossary (DEFAULT_TECHNICAL_TERMS: ROS2, SLAM, LIDAR, PID Controller, etc.)
- [X] T081 [US3] Implement translate handler: Reassemble preserved nodes, store in cache with 5-min TTL
- [X] T082 [US3] Register translation route in `backend/src/index.ts` with rate limiting (5 req/user/min for transformations)
- [ ] T083 [US3] Test translate endpoint: Send chapter, verify Urdu output within 10s (SC-003 translation variant)
- [ ] T084 [US3] Test technical term preservation: Verify ROS, LIDAR, PID remain in English (SC-004: 100% accuracy)
- [ ] T085 [US3] Test code block preservation: Verify Python/C++ code unchanged in translation
- [ ] T086 [US3] Test math equation preservation: Verify LaTeX equations unchanged ($$...$$ syntax)
- [ ] T087 [US3] Test chained transformations: Personalize as Beginner → Translate to Urdu → verify translated version reflects simplified content

### Translation UI Tasks

- [X] T088 [US3] Create UrduToggle component: Integrated into ChapterActions.tsx as "Translate to Urdu" ↔ "Show Original" button
- [X] T089 [US3] Create useTranslation hook in `textbook/src/hooks/useTranslation.ts`: Call /api/translate/urdu, manage toggle state (updated to match backend API: chapterPath, content, translated_content)
- [X] T090 [US3] Add UrduToggle button to ChapterActions component in `textbook/src/components/personalization/ChapterActions.tsx` (integrated with personalize button)
- [X] T091 [US3] Add translation styles to ChapterActions.module.css: `.translateButton` class with pink/red gradient, matching cyber-physical aesthetic
- [ ] T092 [US3] Test translation UI: Click "Translate to Urdu", verify Urdu content appears, button changes to "Show Original"
- [ ] T093 [US3] Test toggle functionality: Click "Show Original", verify English content restored
- [ ] T094 [US3] Test service unavailable: Disconnect backend, click translate, verify error message: "Unable to translate. Please try again."
- [ ] T095 [US3] Test personalize + translate: Personalize as Beginner, then translate, verify Urdu version has simplified explanations

---

## Phase 6: User Story 4 - Research-Validator Subagent (P4)

**Goal**: Content contributors can validate chapter technical accuracy before publishing.

**Why P4**: Internal quality tool for creators, not end-users. Lowest priority for initial release.

**Independent Test**:
1. Submit chapter draft to research-validator
2. Receive structured feedback: incorrect claims flagged, missing citations noted, outdated syntax highlighted
3. Fix issues and resubmit → validation passes

**Acceptance Criteria**: All 4 scenarios from spec.md User Story 4 pass.

### Intelligence Tasks (Research Validator)

- [ ] T096 [US4] Create research-validator skill directory: `.claude/skills/research-validator/`
- [ ] T097 [US4] Write research-validator skill.md following SKILL.md template: Purpose (validate technical accuracy), Process (analyze claims, check citations, verify syntax), Quality Criteria (flag inaccuracies), Examples (validation reports)
- [ ] T098 [US4] Implement research-validator wrapper in `backend/src/skills/research-validator.ts`: Load skill.md, invoke Claude API for validation
- [ ] T099 [US4] Test research-validator: Pass chapter with intentional errors (wrong PID formula, outdated ROS syntax), verify issues detected

### Validation Backend Tasks

- [ ] T100 [US4] Create validation route in `backend/src/routes/validate.ts`: POST /api/validate
- [ ] T101 [US4] Implement validate handler: Invoke research-validator skill, return structured feedback (JSON: {incorrect_claims: [], missing_citations: [], outdated_syntax: []})
- [ ] T102 [US4] Register validation route in `backend/src/index.ts` (no rate limiting for internal tool)
- [ ] T103 [US4] Test validate endpoint: Send draft chapter, verify feedback includes detected issues
- [ ] T104 [US4] Test validation pass: Send high-quality chapter, verify no issues flagged

### Validation UI Tasks (Optional - Internal Tool)

- [ ] T105 [US4] Create ValidationPanel component in `textbook/src/components/validation/ValidationPanel.tsx` (admin-only): Textarea for chapter input, validate button, feedback display
- [ ] T106 [US4] Add validation panel to admin dashboard (if exists) or create standalone route `/admin/validate`
- [ ] T107 [US4] Test validation UI: Paste chapter draft, click validate, verify feedback appears with issues highlighted

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Ensure mobile responsiveness, theme compatibility, and RAG chatbot isolation.

**Independent Test**: All success criteria (SC-001 to SC-010) verified across user stories.

### UI Overhaul Tasks

- [ ] T108 [P] Add neon border styles to sidebar navigation in `textbook/src/css/custom.css`: `.sidebar` classes with blue/purple gradients (#667eea, #764ba2)
- [ ] T109 [P] Add subtle glassmorphism to chapter cards in `textbook/src/css/custom.css`: `.card` classes with backdrop-filter: blur(12px)
- [ ] T110 [P] Add glassmorphism to code blocks in `textbook/src/css/custom.css`: `.code-block` classes (preserve syntax highlighting)
- [ ] T111 Verify light/dark mode compatibility: Toggle theme switcher → all neon/glassmorphism styles adapt correctly
- [ ] T112 Verify CSS namespace isolation: No class name collisions between `.chatbot-*` and `.personalize-*` / `.translate-*`

### Mobile Responsiveness Tasks (SC-007)

- [ ] T113 Test mobile viewport (< 768px): Personalization buttons have 44px minimum tap targets
- [ ] T114 Test mobile viewport: Translate button has 44px minimum tap targets
- [ ] T115 Test mobile viewport: All buttons remain clickable (no z-index conflicts with chatbot FAB)
- [ ] T116 Test mobile viewport: Chapter content remains readable after personalization/translation
- [ ] T117 Test mobile keyboard: Input focus states work correctly on signup form dropdowns

### RAG Chatbot Safety Verification (SC-008)

- [ ] T118 Verify chatbot FAB clickable with personalization buttons visible: Click FAB → chatbot opens
- [ ] T119 Verify chatbot window displays correctly with personalized chapter open: No z-index overlap
- [ ] T120 Verify chatbot functionality unchanged: Send query → typewriter effect works, citations appear
- [ ] T121 Verify chatbot styles untouched: Inspect `textbook/src/components/Chatbot/index.js` (no modifications)
- [ ] T122 Verify Root.js untouched: Inspect `textbook/src/theme/Root.js` (no modifications)
- [ ] T123 Verify chatbot CSS preserved: Inspect `textbook/src/css/custom.css` lines 32-513 (no modifications)

### Verification & Testing Tasks

- [ ] T124 [P] Verify signup completion time: Measure from form open to account created → confirm < 90s (SC-001)
- [ ] T125 [P] Verify authentication success rate: 100 signup attempts → confirm 95% succeed on first try (SC-002)
- [ ] T126 [P] Verify personalization latency: Measure 10 chapter personalizations → confirm < 8s for 95th percentile (SC-003)
- [ ] T127 [P] Verify translation accuracy: Manual review of 20 translated chapters → confirm 100% technical terms preserved (SC-004)
- [ ] T128 [P] Verify complexity reduction: Analyze personalized content for Beginners → confirm 15-25% fewer technical terms, 20-30% more prerequisite explanations (SC-005)
- [ ] T129 [P] Load test: Simulate 100 concurrent users personalizing different chapters → confirm no performance degradation (SC-006)
- [ ] T130 [P] Verify profile updates reflect in session: Update software_background from Beginner → Expert, personalize chapter without reload → confirm Expert-level content (SC-009)
- [ ] T131 User satisfaction survey: Deploy to 10 beta users, collect feedback → target 90% find personalized content more understandable (SC-010)

### Final Integration Tasks

- [ ] T132 Build frontend for production: `npm run build` in `textbook/` → verify no errors
- [ ] T133 Test SSR compatibility: Run `npm run build && npm run serve` → verify no hydration errors, BrowserOnly works
- [ ] T134 Test Vercel deployment: Deploy to Vercel preview → verify all features work (auth, personalization, translation)
- [ ] T135 Create deployment checklist: Document environment variables (DATABASE_URL, ANTHROPIC_API_KEY, JWT_SECRET), migration steps, health checks
- [ ] T136 Write end-user documentation: Update README with features, signup instructions, personalization guide, Urdu translation guide

---

## Phase 8: Landing Page Intelligence Enhancements (Addendum)

**Goal**: Make landing page feature cards "smarter" with interactive actions that trigger personalization features.

**Status**: ✅ COMPLETED (2026-01-13)

**Independent Test**:
1. Click "AI-Powered Learning" card → Navigate to `/docs/intro` → Bulldog automatically opens with welcome message
2. Click "Multilingual (Urdu)" card → Navigate to `/docs/table-of-contents?lang=ur` → Content auto-translates to Urdu
3. Click "Personalized Content" card → Navigate to `/docs/table-of-contents` → See purple banner and ⭐ badges on recommended modules

**Acceptance Criteria**: All 3 smart card actions work seamlessly.

### Smart Card Implementation Tasks

- [x] T137 [LP] Update AI-Powered card in `textbook/src/pages/index.tsx`: Set `from_ai_powered` and `auto_open_bulldog` localStorage flags on click
- [x] T138 [LP] Update BulldogAssistant in `textbook/src/components/BulldogAssistant/index.tsx`: Detect `auto_open_bulldog` flag, force open with personalized welcome message
- [x] T139 [LP] Update Multilingual card in `textbook/src/pages/index.tsx`: Link to `/docs/table-of-contents?lang=ur`, set `trigger_urdu_toc` flag
- [x] T140 [LP] Update UrduToggle in `textbook/src/components/UrduToggle/index.tsx`: Detect `?lang=ur` parameter and `trigger_urdu_toc` flag, auto-translate
- [x] T141 [LP] Add UrduToggle component to `textbook/docs/table-of-contents.md`: Import and render at top of page
- [x] T142 [LP] Update Personalized Content card in `textbook/src/pages/index.tsx`: Link to `/docs/table-of-contents`, set `show_personalized` flag
- [x] T143 [LP] Create PersonalizedTOC banner component in `textbook/src/components/PersonalizedTOC/`: Display user profile summary and recommendation note
- [x] T144 [LP] Create ModuleBadge component in `textbook/src/components/ModuleBadge/`: Show ⭐ "Recommended for You" badges based on user profile
- [x] T145 [LP] Add ModuleBadge to all 4 modules in `textbook/docs/table-of-contents.md`: Module 1-4 headers with dynamic badges
- [x] T146 [LP] Implement recommendation logic in ModuleBadge: Beginner → Modules 1,2; Intermediate → Modules 1,2,3 or 1,3,4; Expert → Modules 3,4
- [x] T147 [LP] Style PersonalizedTOC banner with cyber-physical aesthetic: Purple gradient border, glassmorphism, animated sparkle icon
- [x] T148 [LP] Style ModuleBadge with animations: Purple gradient background, spinning star icon, pulsing glow effect
- [x] T149 [LP] Test AI-Powered card: Click card, navigate to intro, verify Bulldog opens automatically with AI-powered message
- [x] T150 [LP] Test Multilingual card: Click card, navigate to TOC, verify Urdu translation triggers automatically
- [x] T151 [LP] Test Personalized card: Click card (as logged-in user), navigate to TOC, verify badges appear on recommended modules
- [x] T152 [LP] Verify mobile responsiveness: All badges and banner display correctly on 4-7 inch screens

**Notes**:
- These features extend User Story 2 (Personalization) and User Story 3 (Urdu Translation) with enhanced UX
- Implementation completed during hackathon finalization phase
- All components use SSR-safe BrowserOnly wrappers
- localStorage flags ensure cross-page communication without server state

---

## Dependencies & Execution Order

### Story Dependencies

```
Phase 1 (Setup) → BLOCKS ALL
Phase 2 (Foundational) → BLOCKS US1, US2, US3, US4

US1 (Auth) → BLOCKS US2, US3, US4 (authentication required)
US2 (Personalization) → INDEPENDENT from US3, US4
US3 (Translation) → INDEPENDENT from US2, US4 (can combine with US2)
US4 (Validator) → INDEPENDENT from US2, US3
```

### Recommended Execution Order

1. **Week 1**: Phase 1 (Setup) + Phase 2 (Foundational)
2. **Week 2**: Phase 3 (US1 - Auth) → **MVP Release**
3. **Week 3**: Phase 4 (US2 - Personalization)
4. **Week 4**: Phase 5 (US3 - Translation) + Phase 6 (US4 - Validator) in parallel
5. **Week 5**: Phase 7 (Polish & Verification)

### Parallel Execution Opportunities

**Setup Phase**:
- T002, T003, T004, T005 (config files - independent)
- T015, T016, T017, T018 (foundational services - independent)

**US1 (Auth)**:
- T030-T034 (backend auth tests) || T042-T045 (frontend auth tests)

**US2 (Personalization)**:
- T046-T049 (skill creation) || T050-T060 (backend route) || T061-T071 (frontend UI)

**US3 (Translation)**:
- T072-T076 (skill creation) || T077-T087 (backend route) || T088-T095 (frontend UI)

**US4 (Validator)**:
- T096-T099 (skill creation) || T100-T104 (backend route) || T105-T107 (frontend UI)

**Polish Phase**:
- T108, T109, T110 (UI overhaul - independent)
- T124, T125, T126, T127, T128, T129, T130 (verification tests - independent)

---

## Task Summary

**Total Tasks**: 136
**Phase 1 (Setup)**: 14 tasks
**Phase 2 (Foundational)**: 8 tasks
**Phase 3 (US1 - Auth)**: 23 tasks
**Phase 4 (US2 - Personalization)**: 26 tasks
**Phase 5 (US3 - Translation)**: 24 tasks
**Phase 6 (US4 - Validator)**: 12 tasks
**Phase 7 (Polish)**: 29 tasks

**Parallel Tasks**: 42 marked with `[P]`
**User Story Tasks**: 85 marked with `[US#]`
**File Modifications**: 18 existing files enhanced
**New Files Created**: 38 new files

---

## MVP Scope (Minimum Viable Product)

**Phases**: 1 (Setup) + 2 (Foundational) + 3 (US1 - Auth)
**Tasks**: T001-T045 (45 tasks)
**Deliverable**: Users can signup with software/hardware background, login, and update profile
**Value**: Foundation for all personalization features
**Deployment**: Backend + Frontend authentication working end-to-end

---

## Implementation Strategy

1. **Test-Driven where specified**: US1 has explicit test tasks (T030-T034, T042-T045)
2. **Incremental Delivery**: Each user story is independently deployable
3. **File Reuse Priority**: 18 existing files modified vs. 38 new files created (32% reuse rate)
4. **Chatbot Safety**: Tasks T118-T123 verify zero interference (SC-008)
5. **Performance Gates**: Tasks T124-T130 verify all success criteria (SC-001 to SC-009)

---

**Tasks Status**: ✅ READY FOR IMPLEMENTATION
**Next Step**: Execute Phase 1 (T001-T014) to initialize backend and database
**Command**: Start with `T001: Initialize backend project structure`

---

**Generated**: 2026-01-01
**Task Count**: 136 tasks across 7 phases
**Estimated Duration**: 5 weeks (1 week per phase + polish)
**MVP Duration**: 2 weeks (Phase 1-3)
