---
id: 002
title: Clarify Architectural Decisions for RAG Chatbot
stage: spec
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot
branch: 001-rag-chatbot
user: unknown
command: /sp.clarify
labels: ["clarify", "architecture", "rag", "chatbot", "hybrid-search", "chunking", "citations"]
links:
  spec: /mnt/d/Q4_hackathon1/physical_ai_textbook/specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/spec.md
  - specs/001-rag-chatbot/analysis/clarification-analysis.md
  - specs/001-rag-chatbot/checklists/requirements.md
tests:
  - none
---

## Prompt

/sp.clarify

Project: RAG Chatbot for an AI-Native Book

Context:
The chatbot will answer user questions using Retrieval-Augmented Generation (RAG) over the content of an AI-Native textbook (chapters, diagrams, glossary, references).

Please analyze the project specification for:

1. Ambiguous terms
   - What does "AI-Native Book" imply technically?
   - What qualifies as "accurate answer" or "grounded in the book"?
   - What does "support queries" mean (Q&A only, summaries, explanations, citations)?

2. Missing assumptions
   - Target users (students, researchers, general readers)?
   - Supported languages (English only or multilingual)?
   - Expected response style (academic, conversational, concise)?
   - Latency, cost, and deployment environment?
   - Offline vs online indexing?
   - Update frequency of the book content?

3. Incomplete requirements
   - What content is indexed (chapters only, glossary, references, figures)?
   - Chunking strategy expectations?
   - Citation format in answers (inline, footnotes, page/section refs)?
   - How to handle:
     • unanswered questions
     • hallucinations
     • conflicting sections
   - Evaluation criteria (accuracy %, faithfulness, retrieval recall)?
   - Admin / re-index / versioning workflow?

4. Scope conflicts
   - Is the chatbot only for the book or also external knowledge?
   - Is this a learning tutor or a factual retriever?
   - Static textbook vs evolving "living book"?
   - Research assistant vs student help tool?

For each issue:
- Quote or paraphrase the unclear requirement
- Explain the risk or ambiguity
- Propose 1–2 precise clarification questions

Finally:
- Provide a prioritized list of gaps that MUST be resolved before choosing architecture (embeddings, vector DB, retriever, LLM)

## Response snapshot

**Comprehensive Analysis Completed**: Created detailed clarification-analysis.md (16 identified gaps across 4 categories)

**Analysis Categories**:
1. **Ambiguous Terms** (3 issues): AI-Native Book technical implications, "grounded/accurate" operational definitions, query type scope
2. **Missing Assumptions** (7 issues): Response style, content indexing scope, chunking granularity, citation format, evaluation metrics, admin workflow, error handling
3. **Incomplete Requirements** (5 issues): Conflicting information, multi-turn conversations, hybrid search weighting, LLM parameters, partial failures
4. **Scope Conflicts** (4 issues): Book-only vs external knowledge, tutor vs retriever, static vs living book, student vs educator tool

**5 CRITICAL Questions Presented** (blocking architecture):
1. Content Indexing Scope - Which content types to embed?
2. Chunking Strategy - Semantic boundaries? Overlaps?
3. Hybrid Search - Weighting and algorithm?
4. Citation Format - Deep links? Granularity?
5. Response Generation - Temperature? Extractive vs abstractive?

**User Decisions (All Option C or B)**:
- **Q1**: C - Index everything (text, glossary, references, figure captions, code, exercises)
- **Q2**: Part A: B (semantic boundaries, allow variance), Part B: B (10-20% overlap)
- **Q3**: Part A: B (70% semantic / 30% keyword), Part B: A (BM25)
- **Q4**: Part A: C (clickable deep links), Part B: B (group by section)
- **Q5**: Part A: B (temperature 0.3-0.5), Part B: C (hybrid extractive/abstractive)

**Spec Updates Applied**:
- Added "Architectural Decisions (from /sp.clarify)" section with 5 critical decisions (items 9-13)
- Updated FR-001: Expanded to include all content types with content_type metadata
- Added FR-002: New requirement for semantic boundary-aware chunking with overlap strategy
- Updated FR-006: Added 70/30 weighting and BM25 specification
- Updated FR-009: Added Cohere Chat API parameters (temp 0.3-0.5) and hybrid generation approach
- Updated FR-011: Specified clickable deep links with section-level grouping
- Enhanced TextChunk entity: Added content_type, deep_link_url, paragraph_id, overlap_start/end
- Enhanced Response entity: Added grouped citations and generation_temperature
- Updated Soft Constraints: Removed outdated chunking constraint, added incremental re-indexing preference

**Validation Checklist Updated**:
- Marked all critical architectural decisions as resolved
- Listed 6 medium-priority gaps for later (multi-turn, evaluation, conflicting info, admin workflow, response style, error handling)
- Confirmed spec ready for `/sp.plan`

## Outcome

- ✅ Impact: Resolved all blocking architectural ambiguities; spec now contains concrete implementation guidance for embeddings, vector DB schema, retriever, and LLM
- 🧪 Tests: No code tests; analysis framework created for 16 identified gaps
- 📁 Files: Updated spec.md (added 5 architectural decisions, updated 5 FRs, enhanced 2 entities), clarification-analysis.md (full 16-question analysis), requirements.md checklist
- 🔁 Next prompts: `/sp.plan` ready to execute; medium-priority gaps (multi-turn, eval metrics, etc.) can be addressed during planning/implementation
- 🧠 Reflection: Clarification session successfully identified and resolved the 5 most critical architectural blockers. Comprehensive analysis document provides roadmap for addressing remaining 11 medium/low priority gaps during implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None; user provided clear, consistent decisions across all 5 questions
- Graders run and results (PASS/FAIL): Spec quality checklist re-validated - PASS (all architectural decisions now documented)
- Prompt variant (if applicable): Standard /sp.clarify workflow with prioritized question presentation
- Next experiment (smallest change to try): Use the enhanced spec with architectural decisions in `/sp.plan` and validate that planners can design implementation without additional clarifications on chunking, search, or generation strategies
