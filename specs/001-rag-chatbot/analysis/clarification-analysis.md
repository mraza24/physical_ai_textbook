# Clarification Analysis: RAG Chatbot for AI-Native Textbook

**Date**: 2025-12-20
**Feature**: 001-rag-chatbot
**Command**: /sp.clarify
**Purpose**: Identify ambiguities, missing assumptions, incomplete requirements, and scope conflicts before architectural planning

---

## 1. Ambiguous Terms

### 1.1 "AI-Native Book" - Technical Implications

**Spec Reference**: Title and line 1: "RAG Chatbot for AI-Native Textbook"

**Current Language**: The spec uses "AI-Native textbook" without defining what makes it "AI-Native"

**Risk/Ambiguity**:
- Does "AI-Native" mean the book contains interactive elements, dynamic content, or computational examples?
- Does it imply the book has code snippets, Jupyter notebooks, or executable content that should be indexed differently?
- Does it mean the content is generated or curated by AI, affecting how we trust/cite sources?
- Are there special content types (interactive diagrams, 3D models, videos) that need special handling in retrieval?

**Clarification Questions**:
1. **Q1.1a**: What specific content types exist in the "AI-Native textbook" beyond standard text chapters (e.g., code snippets, diagrams, interactive exercises, videos, references, glossary)?
2. **Q1.1b**: Should the chatbot handle different content types differently (e.g., code snippets returned as formatted code blocks, diagrams described textually or linked visually)?

---

### 1.2 "Accurate Answer" and "Grounded in the Book"

**Spec Reference**:
- Line 14: "accurate, contextual answers from book content"
- Line 93 (FR-009): "ensuring responses are grounded only in book content"
- Line 140 (SC-005): "100% of chatbot responses are grounded in book content with source attribution (no hallucinated information)"

**Current Language**: Uses terms like "accurate," "grounded," and "no hallucination" without operational definitions

**Risk/Ambiguity**:
- What constitutes "accurate"? Direct quotes only, or paraphrased explanations allowed?
- Can the chatbot synthesize information from multiple sections to answer complex questions, or must it stick to direct extraction?
- What if the book contradicts itself across sections (e.g., simplified explanation in intro vs. detailed technical explanation later)?
- How do we measure "100% grounded"? What's the test for hallucination vs. reasonable synthesis?

**Clarification Questions**:
1. **Q1.2a**: When answering questions, should the chatbot (A) extract and quote directly from the book, (B) paraphrase/synthesize information while citing sources, or (C) explain concepts in simpler terms using book content as foundation?
2. **Q1.2b**: If the book contains multiple perspectives or evolving explanations of the same concept (e.g., "Chapter 2 introduces concept X simply; Chapter 7 provides advanced treatment"), how should the chatbot reconcile or present this information?

---

### 1.3 "Support Queries" - Scope of Interaction Types

**Spec Reference**:
- Line 12 (User Story 1): "ask 'Can you explain this concept in simpler terms?'"
- Line 21: "What are the main types mentioned here?"
- Line 22: "What is forward kinematics?"
- Line 23: "How does this relate to localization?"

**Current Language**: Examples show explanatory questions, factual lookups, and relational queries, but scope isn't explicitly bounded

**Risk/Ambiguity**:
- Are these the only query types, or should the system also support:
  - Summaries ("Summarize Chapter 3")?
  - Comparisons ("What's the difference between X and Y?")?
  - Multi-step reasoning ("If I have condition A, which approach from Chapter 5 applies?")?
  - Definition lookups ("Define forward kinematics")?
- Should the chatbot provide citations for further reading?
- Should it suggest related topics from the book?

**Clarification Questions**:
1. **Q1.3a**: Beyond explanatory questions, should the chatbot support: (A) Summaries of chapters/sections, (B) Comparisons between concepts, (C) Definitions from a glossary, (D) "Further reading" suggestions from the book?
2. **Q1.3b**: Should responses be optimized for quick factual lookup, or for deeper conceptual understanding with examples from the book?

---

## 2. Missing Assumptions

### 2.1 Expected Response Style and Tone

**Spec Reference**:
- Assumption #5 (line 153): "target users are students and educators"
- No explicit guidance on response tone or style

**Gap**: The spec doesn't specify whether responses should be:
- Academic and formal
- Conversational and friendly
- Concise (definitions/facts only)
- Explanatory (with examples and context)

**Impact**: Response style affects prompt engineering, LLM selection criteria, and user satisfaction

**Clarification Questions**:
1. **Q2.1a**: What response style should the chatbot use: (A) Academic/textbook-like (formal, precise), (B) Conversational/tutor-like (friendly, encouraging), (C) Concise/reference-like (definition + citation only)?
2. **Q2.1b**: Should responses include pedagogical elements like examples, analogies, or step-by-step breakdowns, or focus on direct answers?

---

### 2.2 Content Indexing Scope - What Gets Embedded?

**Spec Reference**:
- Line 85 (FR-001): "embed all published book content"
- Assumption #1 (line 149): "structured markdown or HTML with clear chapter/section hierarchy"

**Gap**: "All published book content" is vague - does this include:
- Main chapter text only?
- Glossary/appendices?
- Figure captions and diagram labels?
- Code comments and docstrings?
- References/bibliography entries?
- Table of contents, index?
- Exercises and solutions?
- Sidebar notes, callouts, warnings?

**Impact**: Affects chunking strategy, embedding volume, retrieval precision, and cost

**Clarification Questions**:
1. **Q2.2a**: Which content types should be indexed for retrieval: (A) Main chapter text only, (B) Main text + glossary + references, (C) Everything including figure captions, code comments, and exercises, (D) Custom list (specify)?
2. **Q2.2b**: Should certain content types (e.g., glossary, code snippets) be indexed separately with special metadata for filtered retrieval?

---

### 2.3 Chunking Strategy and Granularity

**Spec Reference**:
- Line 108-110 (TextChunk entity): "typically 1-3 paragraphs"
- Line 174 (soft constraint): "200-500 tokens per chunk (adjustable based on testing)"

**Gap**: Chunking strategy is mentioned but not defined:
- Should chunks respect semantic boundaries (e.g., end at section breaks, not mid-sentence)?
- How to handle tables, lists, code blocks (keep intact or split)?
- Should chunks overlap to preserve context across boundaries?
- How to chunk non-linear content like glossaries (one entry = one chunk)?

**Impact**: Chunking affects retrieval quality, context coherence, and citation precision

**Clarification Questions**:
1. **Q2.3a**: Should chunks respect semantic boundaries (e.g., complete paragraphs, full list items, intact code blocks) even if it means varying chunk sizes, or enforce strict token limits?
2. **Q2.3b**: Should chunks overlap (e.g., 10-20% overlap) to preserve context across boundaries, or be strictly non-overlapping?

---

### 2.4 Citation Format and Granularity

**Spec Reference**:
- Line 95 (FR-011): "include source references (chapter, section)"
- Line 54 (acceptance scenario): "references like 'See Section 2.3: Kinematics Fundamentals'"

**Gap**: Citation format is shown by example but not specified:
- Should citations include page numbers, line numbers, or paragraph IDs?
- Should citations be inline (within response text) or footnote-style (at the end)?
- For Docusaurus integration, should citations be clickable deep links to specific paragraphs/sections?
- If multiple chunks from the same section are used, how are they cited (one citation per section, or per chunk)?

**Impact**: Affects user trust, verification workflow, and UI implementation

**Clarification Questions**:
1. **Q2.4a**: What citation format should be used: (A) Section-level ("Section 2.3"), (B) Section + paragraph ("Section 2.3, para 4"), (C) Clickable deep links to exact location, (D) Inline references within response text?
2. **Q2.4b**: If multiple chunks from the same section contribute to an answer, should they be cited separately or grouped into one section-level citation?

---

### 2.5 Evaluation Criteria and Metrics

**Spec Reference**:
- Line 136 (SC-001): "95%+ semantic relevance (measured by retrieval accuracy on test query set)"
- Line 140 (SC-005): "100% of chatbot responses are grounded in book content"

**Gap**: Evaluation methods not fully specified:
- What is the "test query set"? Who creates it, how large is it, what topics does it cover?
- How is "semantic relevance" measured (precision@k, recall@k, MRR, nDCG)?
- How is "grounded" measured (human evaluation, automated faithfulness check, source-tracing)?
- What about hallucination detection methods?
- Are there adversarial test cases (trick questions, out-of-scope queries)?

**Impact**: Without clear evaluation, we can't validate if the system meets success criteria

**Clarification Questions**:
1. **Q2.5a**: How should "95%+ semantic relevance" be measured: (A) Precision@5 (top 5 retrieved chunks relevant), (B) Recall (all relevant chunks retrieved), (C) Human evaluation (expert rates relevance), (D) Automated similarity scoring?
2. **Q2.5b**: How should "100% grounded" be validated: (A) Human evaluation (check each response for hallucination), (B) Automated faithfulness scoring (compare response to retrieved chunks), (C) Source-tracing (verify every claim has a cited chunk)?

---

### 2.6 Admin Workflow - Re-indexing and Versioning

**Spec Reference**:
- Line 103 (FR-019): "detect when book content has been updated and trigger re-ingestion"
- Line 145 (SC-010): "within 24 hours of content changes"
- Line 70 (edge case): "detect stale embeddings and trigger re-ingestion"

**Gap**: Admin workflow not specified:
- How does the system detect content changes (webhook, polling, manual trigger)?
- Is re-indexing full (entire book) or incremental (changed sections only)?
- How are multiple book versions handled (students on different editions)?
- Is there a versioning/rollback strategy if re-indexing fails?
- Who has permission to trigger re-indexing?

**Impact**: Affects system reliability, operational complexity, and user experience during updates

**Clarification Questions**:
1. **Q2.6a**: How should content change detection work: (A) Manual trigger by admin, (B) Automated polling (check for changes every N hours), (C) Webhook from Docusaurus build process, (D) Git commit hook?
2. **Q2.6b**: Should re-indexing be (A) Full (re-embed entire book every time), (B) Incremental (detect and re-embed only changed sections), (C) Versioned (maintain multiple book versions simultaneously)?

---

### 2.7 Handling Unanswerable Questions

**Spec Reference**:
- Line 23-24 (acceptance scenario): "I cannot find sufficient information about this topic in the textbook"
- Line 96 (FR-012): "similarity score below threshold"

**Gap**: Threshold and fallback behavior not fully defined:
- What similarity threshold triggers "insufficient information" response (current assumption: 0.7)?
- Should the system suggest related topics from the book when it can't answer directly?
- Should it clarify if the question is ambiguous or out-of-scope vs. genuinely unanswerable?
- Should it log unanswerable questions for content gap analysis?

**Impact**: Affects user experience and helps identify book content gaps

**Clarification Questions**:
1. **Q2.7a**: When a question is unanswerable (low similarity scores), should the chatbot: (A) Simply state "insufficient information," (B) Suggest related topics from the book, (C) Ask clarifying questions, (D) Explain why it couldn't answer?
2. **Q2.7b**: Should unanswerable questions be logged and analyzed to identify content gaps in the textbook?

---

## 3. Incomplete Requirements

### 3.1 Handling Conflicting Information Across Sections

**Spec Reference**: Not addressed in current spec

**Gap**: The textbook may present concepts differently across sections (e.g., simplified intro in Chapter 1, detailed technical treatment in Chapter 8). The spec doesn't address:
- How should the chatbot handle contradictions or evolving complexity?
- Should it present both perspectives?
- Should it favor more recent/advanced sections, or simpler explanations?
- Should it indicate "Chapter X provides a simplified view; see Chapter Y for advanced details"?

**Impact**: Affects answer quality and user confusion

**Clarification Questions**:
1. **Q3.1a**: When the book presents the same concept at different complexity levels (intro vs. advanced), should the chatbot: (A) Default to simpler explanation with option to "learn more," (B) Combine both perspectives in one response, (C) Ask user their level/preference?
2. **Q3.1b**: If the book contains contradictory statements across sections (e.g., updated information in later chapters), should the chatbot: (A) Present both with timestamps/section order, (B) Favor the most recent section, (C) Flag the conflict explicitly?

---

### 3.2 Context Window and Multi-Turn Conversations

**Spec Reference**:
- Line 117 (Query entity): "query_text, selected_text (optional)"
- No mention of conversation history or multi-turn context

**Gap**: The spec doesn't address:
- Does the chatbot maintain conversation history across multiple questions?
- Can users ask follow-up questions ("What about the second type?" after asking "What are the types of X?")?
- Should the chatbot remember previously selected text in the same session?
- How long is conversation history retained?

**Impact**: Affects UX (conversational flow vs. one-shot Q&A) and implementation complexity

**Clarification Questions**:
1. **Q3.2a**: Should the chatbot support multi-turn conversations (remember previous Q&A in session), or treat each query independently?
2. **Q3.2b**: If multi-turn, how many previous turns should be included in context (e.g., last 3 exchanges, entire session up to N tokens)?

---

### 3.3 Hybrid Search - Semantic vs. Keyword Weighting

**Spec Reference**:
- Line 90 (FR-006): "hybrid search combining semantic similarity (via Cohere embeddings) and keyword matching"

**Gap**: "Hybrid search" is mentioned but not defined:
- What's the weighting between semantic and keyword scores (50/50, 70/30, adaptive)?
- What keyword matching algorithm (BM25, TF-IDF, exact match)?
- Should keyword search be a fallback when semantic search fails, or always combined?
- Are there query types that should prefer one over the other (e.g., definitions favor keyword, conceptual questions favor semantic)?

**Impact**: Affects retrieval accuracy and implementation complexity

**Clarification Questions**:
1. **Q3.3a**: What should the weighting be between semantic similarity and keyword matching in hybrid search: (A) 50/50, (B) 70% semantic / 30% keyword, (C) Adaptive based on query type, (D) Semantic-first with keyword fallback?
2. **Q3.3b**: What keyword matching algorithm should be used: (A) BM25, (B) TF-IDF, (C) Exact phrase matching, (D) Fuzzy matching (handle typos)?

---

### 3.4 Response Generation - Temperature and Determinism

**Spec Reference**:
- Line 40 (acceptance scenario): "both responses are semantically consistent (same core information, possibly different wording)"
- Line 93 (FR-009): "generate answers using retrieved chunks as context"

**Gap**: LLM generation parameters not specified:
- Should responses be deterministic (same question = same answer) or allow variation?
- What temperature setting for Cohere Chat API (0 for deterministic, higher for creative)?
- Should the system use few-shot examples to guide response format?
- How strictly should the LLM adhere to retrieved chunks (extractive vs. abstractive)?

**Impact**: Affects reproducibility, answer consistency, and user trust

**Clarification Questions**:
1. **Q3.4a**: Should responses be deterministic (temperature=0, same question always yields identical answer) or allow minor variation for naturalness (temperature=0.3-0.5)?
2. **Q3.4b**: Should the LLM generation be (A) Extractive (mostly quote/paraphrase from chunks), (B) Abstractive (synthesize information in natural language), (C) Hybrid (extract facts, synthesize explanations)?

---

### 3.5 Error Handling - Partial Failures

**Spec Reference**:
- Line 104 (FR-020): "handle API failures (Cohere, Qdrant) gracefully"
- Line 39 (acceptance scenario): "displays a friendly error message"

**Gap**: Error handling for partial failures not addressed:
- What if retrieval succeeds but LLM generation fails (return chunks without answer, or error)?
- What if some chunks fail to embed during ingestion (skip and log, or fail entire batch)?
- What if Qdrant is slow but responsive (wait, timeout, or return partial results)?
- Should there be circuit breakers, retry logic, or degraded mode?

**Impact**: Affects system resilience and user experience

**Clarification Questions**:
1. **Q3.5a**: If retrieval succeeds but LLM generation fails, should the system: (A) Return retrieved chunks as "raw" references, (B) Display error and ask user to retry, (C) Use cached/fallback response template?
2. **Q3.5b**: Should the system implement retry logic for transient API failures (e.g., 3 retries with exponential backoff) or fail immediately and display error?

---

## 4. Scope Conflicts

### 4.1 Book-Only vs. External Knowledge

**Spec Reference**:
- Line 93 (FR-009): "grounded only in book content"
- Line 185 (out of scope): "Non-book-related chat features"
- Constitution: "Chatbot only answers based on selected text or retrieved context; do not hallucinate"

**Conflict/Ambiguity**: The spec clearly states "book-only," but edge cases create gray areas:
- Can the chatbot use external knowledge to explain book concepts in simpler terms (e.g., use common analogies not in the book)?
- Can it use general linguistic knowledge to clarify ambiguous questions before retrieving?
- What about domain-specific terminology not defined in the book (use standard definitions, or refuse to answer)?

**Impact**: Affects prompt design, LLM constraints, and risk of hallucination

**Clarification Questions**:
1. **Q4.1a**: Should the chatbot strictly refuse to use any external knowledge, even for: (A) Explaining concepts in simpler language, (B) Providing common analogies, (C) Defining standard terminology not in the book?
2. **Q4.1b**: If a user asks about something tangentially related to the book (e.g., "How does this compare to approach X not mentioned in the book?"), should the system: (A) Refuse to answer, (B) Answer but cite that info is external, (C) Attempt to find analogous book content?

---

### 4.2 Learning Tutor vs. Factual Retriever

**Spec Reference**:
- Line 12 (User Story 1): "explain this concept in simpler terms" (tutor-like)
- Line 21 (acceptance scenario): "What are the main types mentioned here?" (factual lookup)
- Line 23: "How does this relate to localization?" (conceptual connection)

**Conflict/Ambiguity**: User stories mix tutoring (explanations, simplifications) with factual retrieval (lookups). This creates tension:
- Should the chatbot teach/explain beyond what's in the text, or just retrieve and cite?
- Should it provide mnemonics, analogies, or examples not in the book?
- Should it assess user understanding or just answer questions?

**Impact**: Affects system positioning, LLM prompts, and feature roadmap

**Clarification Questions**:
1. **Q4.2a**: Is the chatbot primarily a (A) Factual retriever (look up and cite book content), (B) Learning tutor (explain concepts, provide examples, simplify), (C) Hybrid (retrieve + explain using book content only)?
2. **Q4.2b**: Should the chatbot provide pedagogical features like: (A) Step-by-step breakdowns, (B) Analogies from the book, (C) Practice questions based on content, (D) None (stick to direct answers)?

---

### 4.3 Static Textbook vs. Living Book

**Spec Reference**:
- Line 152 (assumption): "book content updates are infrequent (monthly or less)"
- Line 103 (FR-019): "detect when book content has been updated"
- Line 145 (SC-010): "within 24 hours of content changes"

**Conflict/Ambiguity**: Spec assumes "infrequent updates" but requires update detection and 24-hour refresh:
- Is this a static textbook (published once, occasional errata) or a living document (continuous updates)?
- Should the chatbot track and communicate book version to users?
- What happens if a student is reading v1.0 but embeddings are from v1.2?
- Should users be able to query specific book versions?

**Impact**: Affects versioning complexity and user experience

**Clarification Questions**:
1. **Q4.3a**: Is the textbook (A) Static (published once, rare updates), (B) Living (frequent updates/additions), (C) Versioned (discrete releases like v1.0, v2.0)?
2. **Q4.3b**: Should users be able to specify which book version they're querying, or always use the latest version?

---

### 4.4 Research Assistant vs. Student Help Tool

**Spec Reference**:
- Line 153 (assumption): "target users are students and educators"
- No explicit user role differentiation in requirements

**Conflict/Ambiguity**: "Students and educators" have different needs:
- Students: Need explanations, examples, summaries for learning
- Educators: Need precise citations, advanced details, fact-checking
- Researchers: Need cross-references, comprehensive coverage, edge cases
- Should the system adapt responses based on user role, or treat everyone the same?

**Impact**: Affects UI design, response style, and feature prioritization

**Clarification Questions**:
1. **Q4.4a**: Should the chatbot tailor responses based on user role: (A) Yes, different response styles for students vs. educators, (B) No, one-size-fits-all, (C) Let users choose "beginner/advanced" mode?
2. **Q4.4b**: Should educators get additional features like: (A) Exhaustive search (all mentions of a topic), (B) Citation export, (C) Confidence scores, (D) None (same features for all users)?

---

## 5. Prioritized Gap List: MUST RESOLVE Before Architecture

### **CRITICAL (Blocking) - Must Resolve Before Planning**

These gaps directly impact architectural decisions (embeddings, vector DB schema, retriever design, LLM prompts):

1. **[CRITICAL] Content Indexing Scope (Q2.2a, Q2.2b)**
   - **Why blocking**: Determines what gets embedded, affects vector DB schema, cost, and retrieval strategy
   - **Architectural impact**: Glossary and code might need separate collections/metadata in Qdrant; affects chunking strategy
   - **Recommended decision**: Define explicit list of content types to index (e.g., main text + glossary + code snippets, but exclude TOC and index)

2. **[CRITICAL] Chunking Strategy (Q2.3a, Q2.3b)**
   - **Why blocking**: Affects embedding generation, retrieval quality, and context coherence
   - **Architectural impact**: Chunking algorithm, overlap strategy, token limits
   - **Recommended decision**: Semantic chunking with 200-500 token target, allow variance for integrity (e.g., keep code blocks intact), 10% overlap

3. **[CRITICAL] Hybrid Search Implementation (Q3.3a, Q3.3b)**
   - **Why blocking**: Core retrieval mechanism, affects Qdrant queries and ranking
   - **Architectural impact**: BM25 implementation, score fusion algorithm
   - **Recommended decision**: 70% semantic / 30% BM25 keyword, adaptive based on query length (short queries favor keyword, long favor semantic)

4. **[CRITICAL] Citation Format and Granularity (Q2.4a, Q2.4b)**
   - **Why blocking**: Affects chunk metadata schema in Qdrant and response formatting
   - **Architectural impact**: Metadata fields (chapter, section, paragraph_id, deep_link_url), UI implementation
   - **Recommended decision**: Section-level citations with clickable deep links to Docusaurus paragraphs; group multiple chunks from same section

5. **[CRITICAL] Response Generation Style (Q3.4a, Q3.4b)**
   - **Why blocking**: Affects LLM prompt design and reproducibility
   - **Architectural impact**: Cohere API parameters (temperature, max_tokens), prompt templates
   - **Recommended decision**: Hybrid approach (temperature=0.3 for slight variation), abstractive synthesis constrained by retrieved chunks

---

### **HIGH PRIORITY - Should Resolve Before Implementation**

These gaps don't block architecture design but should be clarified before coding:

6. **[HIGH] Multi-Turn Conversation Support (Q3.2a, Q3.2b)**
   - **Why important**: Affects session management, context window, and UX
   - **Impact**: Session state storage in Postgres, context assembly for LLM
   - **Recommended decision**: Support multi-turn with last 3 Q&A pairs in context (up to 1000 tokens)

7. **[HIGH] Evaluation Metrics and Test Set (Q2.5a, Q2.5b)**
   - **Why important**: Can't validate system without clear metrics
   - **Impact**: Test harness design, acceptance criteria validation
   - **Recommended decision**: Create 50-question test set covering various topics; measure Precision@5 for retrieval, human eval for groundedness

8. **[HIGH] Conflicting Information Handling (Q3.1a, Q3.1b)**
   - **Why important**: Affects answer quality and user trust
   - **Impact**: Prompt design, chunk ranking/deduplication
   - **Recommended decision**: Default to simpler explanations with "See Section X.Y for advanced details" links

9. **[HIGH] Admin Re-indexing Workflow (Q2.6a, Q2.6b)**
   - **Why important**: Affects operational complexity and reliability
   - **Impact**: Ingestion pipeline, change detection, versioning
   - **Recommended decision**: Manual trigger for hackathon, webhook for production; incremental re-indexing

---

### **MEDIUM PRIORITY - Can Defer to Implementation**

These gaps can be addressed during implementation without major rework:

10. **[MEDIUM] Response Style and Tone (Q2.1a, Q2.1b)**
    - **Impact**: Prompt engineering, user satisfaction
    - **Recommended decision**: Conversational/tutor-like with examples from book

11. **[MEDIUM] Unanswerable Question Handling (Q2.7a, Q2.7b)**
    - **Impact**: User experience, content gap analysis
    - **Recommended decision**: Suggest related topics + log for analysis

12. **[MEDIUM] Error Handling - Partial Failures (Q3.5a, Q3.5b)**
    - **Impact**: Resilience, UX
    - **Recommended decision**: 3 retries with backoff; if retrieval OK but LLM fails, return chunks as references

13. **[MEDIUM] Query Type Scope (Q1.3a, Q1.3b)**
    - **Impact**: Feature scope, user expectations
    - **Recommended decision**: Support explanations, definitions, comparisons; defer summaries to future

---

### **LOW PRIORITY - Clarify If Time Permits**

These gaps have minimal architectural impact and can be addressed iteratively:

14. **[LOW] User Role Differentiation (Q4.4a, Q4.4b)**
    - **Impact**: UI/UX, feature richness
    - **Recommended decision**: One-size-fits-all for hackathon, role-based later

15. **[LOW] Book Versioning Strategy (Q4.3a, Q4.3b)**
    - **Impact**: Versioning complexity
    - **Recommended decision**: Always use latest version for hackathon, version support later

16. **[LOW] External Knowledge Boundaries (Q4.1a, Q4.1b)**
    - **Impact**: Prompt constraints, hallucination risk
    - **Recommended decision**: Strict book-only for factual claims; allow linguistic/explanatory knowledge for clarity

---

## Summary: Next Steps

**Immediate Actions**:
1. **Resolve CRITICAL items (1-5)** before running `/sp.plan` - these directly impact architecture
2. **Document decisions** in spec.md Assumptions section
3. **Update functional requirements** with clarified details (chunking, hybrid search, citations)
4. **Create test query set** (50 questions) for evaluation (item 7)

**Recommended Approach**:
Present the 5 CRITICAL questions (items 1-5) to the user as a focused clarification session. Use the AskUserQuestion tool to gather responses, then update the spec and proceed to planning.

**Risk if Not Resolved**:
- Chunking and indexing decisions made incorrectly → poor retrieval quality, costly re-work
- Hybrid search implementation unclear → suboptimal ranking, missed relevant content
- Citation format undefined → UI/UX mismatch, re-implementation needed
- Response generation style unclear → inconsistent answers, user dissatisfaction
