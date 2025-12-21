# ADR-005: English-Only Scope for Hackathon

**Status**: Accepted
**Date**: 2025-12-20
**Deciders**: Architecture Team
**Feature**: 001-rag-chatbot

---

## Context

The RAG chatbot needs to define its language support scope for both book content and user queries. Language support directly impacts development complexity, API costs, user accessibility, and system architecture.

**Key Requirements**:
- Support students and educators interacting with Physical AI textbook
- Deliver working chatbot within hackathon timeline (24-48 hours)
- Work within free tier budget constraints (no paid translation APIs)
- Provide clear error messaging when system cannot handle requests
- Design for future extensibility (multilingual support post-hackathon)

**Book Content Language**:
- Physical AI textbook is written entirely in English
- No multilingual versions currently exist
- All chapters, glossary, code comments, references are in English

**User Base Assumptions**:
- Primary audience: Students and educators in AI/robotics programs
- Expected to have English proficiency (technical field, international language)
- Edge case: Non-English speakers may attempt to use chatbot

**Constraints**:
- Hackathon timeline: 24-48 hours total development time
- Free tier only: No budget for paid translation APIs (Google Translate $20/1M chars, DeepL €4.99/mo)
- Cohere embeddings: English-optimized model (`embed-english-v3.0`)
- Testing scope: Limited time for multilingual query testing

---

## Decision

**We will support English-only for both book content and user queries in hackathon scope.**

This decision includes:
- **Book Content**: Index only English text (already constraint, no translation needed)
- **User Queries**: Accept only English queries, reject non-English with clear error message
- **Language Detection**: Simple heuristic (high non-ASCII character ratio) or library (langdetect)
- **Error Messaging**: "I can only answer questions in English. Please rephrase your question in English."
- **Future Extensibility**: Design modular architecture to add translation layer post-hackathon

---

## Rationale

### Why English-Only for Hackathon

**Development Simplicity**:
- **No translation layer needed**: Saves 4-6 hours of development time
  - No Google Translate/DeepL API integration
  - No query translation logic (non-English → English)
  - No response translation logic (English → user's language)
- **Single language testing**: No multilingual query test cases needed
- **Simpler error handling**: Single code path for query processing

**Book Content Constraint**:
- **Textbook is English**: No multilingual book versions exist
- **No translation needed**: Book content already in target language
- **Embedding optimization**: Cohere `embed-english-v3.0` optimized for English text
  - Better semantic understanding for English queries
  - No multilingual embedding model needed (larger, slower)

**User Base Alignment**:
- **Technical field**: AI/robotics education typically conducted in English
- **Primary audience**: Students/educators expected to have English proficiency
- **Acceptable tradeoff**: Hackathon scope prioritizes core functionality over inclusivity

**Cost Avoidance**:
- **Free tier budget**: Translation APIs add cost (Google Translate $20/1M chars)
- **API call overhead**: Each query requires 2 translation calls (query in, response out)
- **Embedding cost**: Multilingual models may have different pricing

**Future Extensibility**:
- **Modular design**: Translation layer can be added as middleware post-hackathon
- **Clear extension point**: Query validation → translation → embedding
- **Demand-driven**: Add multilingual support if user requests warrant it

### Why Simple Language Detection

**Heuristic Approach** (Hackathon):
- Reject queries with >30% non-ASCII characters
- Fast, no external library needed
- Catches most non-English queries (Chinese, Arabic, Russian, etc.)

**Library Approach** (Alternative):
- Use `langdetect` or `fasttext` for language identification
- More accurate, but adds dependency
- Overkill for hackathon scope

**Tradeoff Accepted**:
- False positives: Technical terms with accents (e.g., "Poincaré map") may trigger rejection
- False negatives: English with poor grammar may pass detection
- Mitigation: Clear error message allows user to rephrase

---

## Alternatives Considered

### Alternative 1: Multilingual Queries → English Responses

**Approach**: Accept queries in any language, translate to English, respond in English

**Pros**:
- Broader user accessibility (non-English speakers can query)
- Leverages existing English book content
- Simpler than full multilingual support (no response translation)

**Cons**:
- ❌ **Translation API cost**: Google Translate ~$20/1M chars (exceeds free tier)
- ❌ **Latency overhead**: +200-500ms per query for translation API call
- ❌ **Translation quality risk**: Technical terms may mistranslate ("sensor fusion" → incorrect equivalents)
- ❌ **Development time**: 2-3 hours to integrate translation API
- ❌ **Response mismatch**: User queries in Spanish, receives English answer (UX issue)

**Verdict**: Rejected. Cost and development time outweigh benefit for hackathon scope.

---

### Alternative 2: Full Multilingual Support (Queries + Responses)

**Approach**: Accept queries in any language, translate to English, retrieve, translate response back

**Pros**:
- Maximum accessibility (global user base)
- Best UX (user queries and receives responses in native language)
- Comprehensive solution

**Cons**:
- ❌ **Highest cost**: 2 translation API calls per query (query in, response out)
- ❌ **Highest latency**: +400-1000ms per query (2 translation calls)
- ❌ **Complexity**: Translation error handling, language detection, response formatting
- ❌ **Development time**: 6-8 hours (integration + testing)
- ❌ **Not hackathon viable**: Too much scope for 24-48 hour timeline

**Verdict**: Rejected. Scope too large for hackathon, better as post-hackathon feature.

---

### Alternative 3: No Language Detection (Accept All Languages)

**Approach**: Accept any query, let Cohere embeddings handle it (hope for cross-lingual matching)

**Pros**:
- Zero development effort (no language detection logic)
- No false positives rejecting valid queries
- Relies on embedding model's cross-lingual capabilities

**Cons**:
- ❌ **Poor retrieval quality**: Cohere `embed-english-v3.0` not optimized for non-English
- ❌ **Confusing UX**: Non-English query returns English chunks (or no results)
- ❌ **No error guidance**: User doesn't know why query failed
- ❌ **Silent failure**: User may think system is broken, not language limitation

**Verdict**: Rejected. Terrible UX, no user guidance, fails >95% accuracy target for non-English queries.

---

### Alternative 4: Multilingual Book Content (Translate Book)

**Approach**: Translate entire textbook to multiple languages, index all versions

**Pros**:
- Native language support for book content
- No query translation needed
- Better semantic matching (query and content in same language)

**Cons**:
- ❌ **Not applicable**: Textbook only exists in English (no translations available)
- ❌ **Massive scope**: Translating entire textbook requires weeks/months
- ❌ **Quality risk**: Technical content difficult to translate accurately
- ❌ **Storage cost**: 10x embeddings (10k chunks × 10 languages = 100k vectors)
- ❌ **Beyond hackathon**: Post-hackathon consideration if translations become available

**Verdict**: Rejected. Not applicable (no multilingual book), far beyond hackathon scope.

---

## Consequences

### Positive Outcomes

1. **Faster Development**: 4-6 hours saved by avoiding translation integration
   - No API setup, no translation logic, no multilingual testing
   - Focus on core RAG functionality (retrieval, generation, citations)

2. **Lower Cost**: Zero translation API costs
   - Stays within free tier budget
   - No ongoing translation API expenses

3. **Simpler Architecture**: Single language code path
   - Fewer error states to handle
   - Easier to test and debug
   - Clearer system boundaries

4. **Better Semantic Quality**: English-optimized embeddings
   - Cohere `embed-english-v3.0` designed for English text
   - Higher retrieval accuracy for English queries (>95% target achievable)

5. **Clear Extension Point**: Modular design supports future translation layer
   - Query validation → [translation middleware] → embedding
   - Can add post-hackathon without major refactoring

### Negative Outcomes & Mitigations

1. **Limited User Accessibility**:
   - **Impact**: Non-English speakers cannot use chatbot
   - **Scope**: Acceptable for hackathon demo, not production
   - **Mitigation**: Clear error message guides user to rephrase in English
   - **Future**: Add translation layer post-hackathon if demand exists

2. **False Positives in Language Detection**:
   - **Impact**: Technical terms with accents (Poincaré, naïve) may trigger rejection
   - **Mitigation**: Use >30% non-ASCII threshold (not >5%), reduces false positives
   - **User feedback**: "Please rephrase in English" allows retry
   - **Future**: Use library-based detection (`langdetect`) for higher accuracy

3. **Perception of Exclusivity**:
   - **Impact**: Non-English users may perceive system as inaccessible
   - **Mitigation**: Documentation states "English-only for hackathon demo"
   - **Future**: Roadmap includes multilingual support post-hackathon

4. **Missed User Insights**:
   - **Impact**: Cannot observe non-English query patterns or demand
   - **Mitigation**: Log rejected queries (anonymized) to measure demand
   - **Future**: Prioritize translation layer if significant non-English traffic observed

### Implementation Impact

**Files Affected**:
- `app/services/query_validator.py`: Language detection logic
- `app/api/routes/chat.py`: Reject non-English queries with error message
- `app/models/response.py`: Error response schema
- `rag-backend/.env`: `SUPPORTED_LANGUAGES=en` (future: `en,es,fr,zh`)

**Language Detection Implementation**:
```python
def is_english(query_text: str) -> bool:
    """
    Simple heuristic: Reject if >30% non-ASCII characters.
    Catches most non-English languages (Chinese, Arabic, Russian, etc.)
    May have false positives for technical terms with accents.
    """
    total_chars = len(query_text)
    non_ascii_chars = sum(1 for c in query_text if ord(c) > 127)
    non_ascii_ratio = non_ascii_chars / total_chars if total_chars > 0 else 0

    return non_ascii_ratio < 0.3  # Allow up to 30% non-ASCII (handles accents)

# Alternative: Library-based (more accurate, adds dependency)
from langdetect import detect

def is_english(query_text: str) -> bool:
    try:
        detected_lang = detect(query_text)
        return detected_lang == 'en'
    except:
        return True  # Default to accepting if detection fails
```

**Error Response Format**:
```python
# app/models/response.py
class ChatResponse(BaseModel):
    answer: Optional[str]
    citations: List[Citation]
    error: Optional[str]
    error_code: Optional[str]  # "LANGUAGE_NOT_SUPPORTED"

# app/api/routes/chat.py
if not is_english(query_text):
    return ChatResponse(
        answer=None,
        citations=[],
        error="I can only answer questions in English. Please rephrase your question in English.",
        error_code="LANGUAGE_NOT_SUPPORTED"
    )
```

**Future Extension Point** (Post-Hackathon):
```python
# app/services/translation_service.py (future)
class TranslationService:
    def translate_to_english(self, text: str, source_lang: str) -> str:
        """Translate query to English for retrieval"""
        # Google Translate or DeepL API call
        pass

    def translate_from_english(self, text: str, target_lang: str) -> str:
        """Translate response back to user's language"""
        pass

# Middleware integration (future)
detected_lang = detect_language(query_text)
if detected_lang != 'en':
    query_text = translation_service.translate_to_english(query_text, detected_lang)

# ... retrieval and generation ...

if detected_lang != 'en':
    answer = translation_service.translate_from_english(answer, detected_lang)
```

**Testing Requirements**:
- **Unit Tests**:
  - Test language detection (English passes, non-English fails)
  - Test error response format (correct message, error_code)
  - Test edge cases (empty query, mixed language, technical terms with accents)
- **Integration Tests**:
  - English query: "What is SLAM?" → succeeds, retrieves content
  - Non-English query: "¿Qué es SLAM?" → fails with clear error message
  - Mixed query: "What is Poincaré map?" → succeeds (below 30% threshold)
- **User Acceptance**:
  - Verify error message is clear and actionable
  - Confirm no false positives for common technical terms

---

## References

- **Plan Document**: `specs/001-rag-chatbot/plan.md` (ADR-005: English-Only Scope)
- **Specification**: `specs/001-rag-chatbot/spec.md` (Edge Case: "How does the system handle questions in languages other than the book's language?", Assumption #2: English language)
- **Constitution**: `.specify/memory/constitution.md` (Section V: Maintainability - modular design for future extensibility)
- **Related ADRs**:
  - ADR-001 (Cohere Embeddings): English-optimized model (`embed-english-v3.0`)
  - ADR-004 (Hybrid Search): BM25 tokenization assumes English text
- **Research**:
  - langdetect library: https://github.com/Mimino666/langdetect
  - Google Cloud Translation Pricing: https://cloud.google.com/translate/pricing
  - Cohere Multilingual Embeddings: https://docs.cohere.com/docs/multilingual-language-models

---

## Acceptance Criteria

This ADR is accepted when:
- [x] Decision clusters language support scope (book content + queries + detection + error messaging)
- [x] At least one alternative explicitly listed (4 alternatives documented)
- [x] Clear pros and cons for chosen approach and alternatives
- [x] Consequences cover both positive (speed, cost, simplicity) and negative (accessibility, false positives) outcomes
- [x] References link back to plan, spec, constitution, and related ADRs
- [x] Implementation impact specified (detection logic, error response, future extension point)

---

## Review Notes

- **Analyzed**: Decision is architecturally significant (impacts query pipeline, user accessibility, future extensibility)
- **Measured**: PASS - Clusters language support scope (book + queries + detection + messaging), lists 4 alternatives, comprehensive tradeoff analysis
- **Risk**: Low - Acceptable for hackathon scope, clear mitigation path (translation layer post-hackathon)
- **Dependency**: Related to ADR-001 (English embeddings), ADR-004 (English BM25 tokenization)
- **Future Work**: Add translation middleware if non-English demand observed (>10% rejected queries)
