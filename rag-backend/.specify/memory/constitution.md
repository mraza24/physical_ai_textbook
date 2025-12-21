# RAG Chatbot for AI-native Textbook - Constitution

## Core Principles

### I. Accuracy
**All chatbot responses must be grounded exclusively in book content or user-selected text.**
- No hallucination: if context is insufficient, explicitly state this to the user
- Responses must cite or reference the source material when possible
- Only published book content can be referenced; no external sources

### II. Reliability
**All embeddings and retrieval must reflect the most up-to-date book version.**
- Embedding pipeline must re-run when book content is updated
- Vector database must maintain consistency with current book state
- Semantic + keyword search combined to ensure 95%+ retrieval accuracy in test cases
- System must handle concurrent users without degradation

### III. Transparency
**Responses should indicate if context is insufficient or confidence is low.**
- When retrieved context doesn't fully answer the query, acknowledge limitations
- User-selected text overrides default retrieval context
- Clear error messages when system cannot fulfill request

### IV. Security & Privacy
**No sensitive user data stored beyond session scope.**
- User session tracking limited to metadata required for functionality
- API keys (Cohere, Neon, Qdrant) must be stored in environment variables, never hardcoded
- All database queries must use parameterized statements (no SQL injection)
- Rate limiting enforced to prevent abuse

### V. Maintainability
**System architecture must allow future integration of new LLMs and book content.**
- Modular design: embedding generation, storage, retrieval, and chat layers decoupled
- Clear interfaces for swapping LLM providers (e.g., Cohere → OpenAI → future models)
- Configuration-driven approach for model selection and parameters
- Comprehensive documentation for onboarding new contributors

### VI. Testability
**All endpoints must be tested with sample book content before live deployment.**
- Unit tests for each backend endpoint (ingestion, retrieval, query)
- Integration tests for end-to-end RAG pipeline
- Test fixtures with representative book content
- Performance benchmarks for retrieval latency and accuracy

## Technology Standards

### Stack Requirements
- **Embeddings**: Cohere API for text embeddings (NOT OpenAI)
- **Vector Database**: Qdrant Cloud Free Tier for chunk storage and retrieval
- **Backend**: FastAPI for all API endpoints (ingestion, retrieval, query)
- **Relational Database**: Neon Serverless Postgres for metadata and user session tracking
- **Chat UI**: OpenAI ChatKit SDK integrated into Docusaurus book interface

### API Constraints
- Responses must be under 500 tokens for quick UI display
- All endpoints must return proper HTTP status codes and error messages
- Request/response formats must be documented (OpenAPI/Swagger)
- CORS configured appropriately for Docusaurus frontend

### RAG Behavior
- **Retrieval**: Combine semantic search (via Cohere embeddings) + keyword search
- **Top N**: Return top N relevant chunks (N configurable, default 3-5)
- **Context window**: Pass retrieved chunks + user query to LLM for answer generation
- **Fallback**: If no relevant chunks found (similarity < threshold), return "context insufficient" message

## Quality Gates

### Pre-Deployment Checklist
- [ ] All endpoints tested with sample book content
- [ ] Semantic search achieves 95%+ accuracy on test queries
- [ ] Backend handles multiple simultaneous users without errors
- [ ] Chat UI correctly streams answers in Docusaurus interface
- [ ] No API keys or secrets exposed in code
- [ ] Error handling covers common failure modes (API timeout, DB connection loss, etc.)

### Performance Benchmarks
- Retrieval latency: < 500ms for top 5 chunks
- End-to-end query response: < 3s including LLM generation
- Concurrent users supported: minimum 10 simultaneous queries

### Code Quality
- All functions have type hints (Python) or TypeScript types
- Docstrings for public functions
- Linting passes (ruff/black for Python, eslint/prettier for TypeScript)
- No commented-out code in production

## Development Workflow

### Feature Development
1. **Spec-first**: Document requirements in `specs/<feature>/spec.md`
2. **Plan**: Create architecture plan in `specs/<feature>/plan.md`
3. **Tasks**: Break down into testable tasks in `specs/<feature>/tasks.md`
4. **Implement**: Follow smallest viable diff principle
5. **Test**: Verify against acceptance criteria
6. **Document**: Update relevant docs and ADRs

### Testing Strategy
- **Unit tests**: Individual functions and endpoints
- **Integration tests**: RAG pipeline end-to-end
- **Manual QA**: Test in Docusaurus UI before release
- **Regression tests**: Maintain test suite as book content evolves

### Version Control
- Feature branches from `main`
- Pull requests require review and passing tests
- Commits follow conventional commit format
- No direct pushes to `main`

## Constraints

### Hard Constraints (Non-Negotiable)
- Must use Cohere API for embeddings (not OpenAI)
- Responses must be under 500 tokens
- Only published book content can be referenced
- No sensitive user data stored beyond session

### Soft Constraints (Preferred)
- Prefer FastAPI features over custom middleware
- Prefer Pydantic models for request/response validation
- Prefer async/await for I/O operations
- Prefer environment variables over config files for secrets

## Future Extensibility

### Planned Integrations
- **Subagents**: Architecture should support future subagent / multi-agent workflows
- **Reusable Intelligence**: Design for knowledge reuse across multiple books/domains
- **Multi-LLM**: Ability to switch between Cohere, OpenAI, Anthropic, local models
- **Enhanced Retrieval**: Support for multi-modal content (diagrams, code snippets, equations)

### Migration Path
- When adding new LLM providers, implement adapter pattern
- When adding new book content, ensure embedding pipeline is idempotent
- When scaling, ensure vector DB and relational DB can be migrated to higher tiers

## Governance

### Constitution Authority
- This constitution supersedes project-level decisions
- All code reviews must verify compliance with these principles
- Exceptions require documented justification and approval

### Amendment Process
1. Propose amendment with rationale and impact analysis
2. Discuss with team/stakeholders
3. Update constitution with version increment
4. Create ADR documenting the change
5. Update dependent specs, plans, and code

### Success Criteria (Final Validation)
- Chatbot answers accurately from book content
- Semantic search retrieves relevant chunks 95%+ accuracy in test cases
- Backend endpoints function without errors and scale to multiple simultaneous users
- Chat UI correctly streams answers in the Docusaurus book interface
- System ready for future Subagents / reusable intelligence integration

---

**Version**: 1.0.0
**Ratified**: 2025-12-20
**Last Amended**: 2025-12-20
