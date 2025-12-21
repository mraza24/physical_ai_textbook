---
id: 008
title: Document English-Only Scope ADR
stage: plan
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.adr
labels: ["adr", "language-support", "english-only", "scope", "accessibility", "architecture"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: /mnt/d/Q4_hackathon1/physical_ai_textbook/rag-backend/history/adr/005-english-only-scope-for-hackathon.md
  pr: null
files:
  - history/adr/005-english-only-scope-for-hackathon.md
tests:
  - none
---

## Prompt

/sp.adr English-Only Scope for Hackathon

## Response snapshot

**ADR-005 Created**: English-Only Scope for Hackathon

**Decision**: Support English-only for both book content and user queries in hackathon scope

**Context**:
- Need to define language support scope for chatbot (book content + user queries)
- Requirements: Hackathon timeline (24-48 hrs), free tier budget, clear error messaging, future extensibility
- Book content: Physical AI textbook written entirely in English
- User base: Students/educators in AI/robotics (expected English proficiency)
- Constraints: No budget for translation APIs ($20/1M chars), Cohere embeddings English-optimized

**Rationale** (3 components):
1. **Development Simplicity**: Saves 4-6 hours (no translation API integration, single language testing, simpler error handling)
   - No Google Translate/DeepL setup
   - No multilingual query test cases
   - Single code path for query processing

2. **Book Content Constraint + User Alignment**: Textbook is English (no translation needed), Cohere `embed-english-v3.0` optimized for English
   - Primary audience: Technical field (AI/robotics education typically English)
   - Acceptable tradeoff: Hackathon scope prioritizes core functionality over inclusivity

3. **Cost Avoidance + Future Extensibility**: Free tier budget (translation APIs add cost), modular design supports post-hackathon translation layer
   - Clear extension point: Query validation → [translation middleware] → embedding
   - Demand-driven: Add multilingual if user requests warrant it

**Language Detection**:
- Simple heuristic: Reject queries with >30% non-ASCII characters (catches Chinese, Arabic, Russian)
- Error message: "I can only answer questions in English. Please rephrase your question in English."
- Alternative: Library-based (`langdetect`) more accurate but adds dependency

**Tradeoffs**:
- **Pros**: Faster development (4-6 hrs saved), lower cost ($0 translation), simpler architecture (single language path), better semantic quality (English-optimized embeddings), clear extension point
- **Cons**: Limited accessibility (non-English speakers excluded, acceptable for hackathon), false positives (technical terms with accents like "Poincaré" may trigger rejection, mitigated by 30% threshold), perception of exclusivity (mitigated by documentation), missed user insights (log rejected queries to measure demand)

**Alternatives Considered** (4):
1. **Multilingual Queries → English Responses**: Accept any language, translate to English, respond in English, but translation API cost ($20/1M chars), latency (+200-500ms), translation quality risk (technical terms), development time (2-3 hrs), response mismatch UX (rejected: cost + time)
2. **Full Multilingual (Queries + Responses)**: Translate query in, response out, maximum accessibility, but highest cost (2 API calls/query), highest latency (+400-1000ms), complexity, 6-8 hrs development (rejected: too large for hackathon)
3. **No Language Detection**: Accept all, let embeddings handle it, but poor retrieval (Cohere not optimized for non-English), confusing UX (English chunks for non-English query), no error guidance, silent failure (rejected: terrible UX, fails 95% accuracy)
4. **Multilingual Book Content**: Translate entire textbook, native language support, but not applicable (book only in English), massive scope (weeks/months), quality risk, 10x storage (100k vectors), beyond hackathon (rejected: not applicable)

**Consequences**:
- **Positive**: Faster development (4-6 hrs saved), lower cost ($0), simpler architecture, better semantic quality (>95% achievable), clear extension point (modular design)
- **Negative**: Limited accessibility (non-English users excluded, acceptable for hackathon), false positives (>30% non-ASCII threshold reduces risk), perception of exclusivity (documentation states hackathon demo), missed insights (log rejected queries)

**Implementation Impact**:
- Files: `query_validator.py` (language detection), `routes/chat.py` (reject with error), `response.py` (error schema), `.env` (`SUPPORTED_LANGUAGES=en`)
- Detection logic: `non_ascii_ratio < 0.3` (simple heuristic) or `langdetect.detect(query) == 'en'` (library-based)
- Error response: `ChatResponse(error="I can only answer questions in English...", error_code="LANGUAGE_NOT_SUPPORTED")`
- Future extension: `TranslationService` middleware (translate_to_english, translate_from_english) for post-hackathon
- Testing: Unit (English passes, non-English fails, edge cases), integration (English query succeeds, non-English fails with error, mixed query like "Poincaré map" succeeds), user acceptance (clear error message)

**Related ADRs**:
- ADR-001 (Cohere Embeddings): English-optimized model (`embed-english-v3.0`)
- ADR-004 (Hybrid Search): BM25 tokenization assumes English text

## Outcome

- ✅ Impact: Formalized language support scope; defines query pipeline constraints, user accessibility, and future extensibility path
- 🧪 Tests: Defined 3 test categories (unit: detection logic, integration: query scenarios, user acceptance: error clarity)
- 📁 Files: Created history/adr/005-english-only-scope-for-hackathon.md (4.8KB)
- 🔁 Next prompts: Document remaining ADR (indexing strategy), then /sp.tasks
- 🧠 Reflection: ADR clusters language support scope (book content + queries + detection + error messaging + future extension), with 4 alternatives and comprehensive tradeoff analysis. English-only justified by hackathon timeline, cost constraints, and book content constraint, with clear post-hackathon extension path for multilingual support.

## Evaluation notes (flywheel)

- Failure modes observed: None; decision well-documented in plan.md and spec.md (edge case clarification, assumption #2)
- Graders run and results (PASS/FAIL):
  - Analyze: PASS - Clusters language support scope (not over-granular), lists 4 alternatives with detailed rationale
  - Measure: PASS - Architecturally significant (impacts query pipeline, user accessibility, future extensibility), explicit alternatives, clear pros/cons, comprehensive detail with detection logic and future extension code
- Prompt variant (if applicable): N/A - standard /sp.adr workflow
- Next experiment (smallest change to try): Create final ADR (indexing strategy), then validate all 6 ADRs for consistency before /sp.tasks
