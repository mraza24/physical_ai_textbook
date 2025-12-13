<!--
Sync Impact Report:

- **Version change**: 1.1.0 → 1.1.1
- **Bump rationale**: PATCH - Verification pass confirming constitution alignment after
  sp.analyze identified spec-driven workflow compliance issues. No substantive changes
  to principles, standards, or governance.
- **Modified principles**: None
- **Added sections**: None
- **Removed sections**: None
- **Templates requiring updates**:
  ✅ .specify/templates/plan-template.md - Constitution Check section verified (lines 30-38)
  ✅ .specify/templates/spec-template.md - No constitution references requiring updates
  ✅ .specify/templates/tasks-template.md - No constitution references requiring updates
  ✅ .claude/commands/*.md - All 9 command files verified, no outdated references
- **Follow-up TODOs**: None
- **Verification notes**: Constitution validated after sp.analyze detected missing spec.md
  in feature 006-tasks. Spec-driven workflow principle (Specs → Tasks → Outputs) confirmed
  as non-negotiable requirement.
-->
# AI/Spec-Driven Book Creation: Physical AI & Humanoid Robotics

**Project**: Write a technical textbook using Docusaurus, automate content generation
with Spec-Kit Plus and Claude Code, and deploy the final book to GitHub Pages.

## Core Principles

### Accuracy

All technical, conceptual, and architectural claims MUST be verified through official
documentation, RFCs, published standards, or authoritative sources. No unverified or
speculative content is permitted.

**Rationale**: Technical accuracy is non-negotiable in educational material. Readers
trust textbooks to provide correct information that forms the foundation of their
understanding and practice.

### Clarity

Content MUST be understandable to intermediate-to-advanced readers in AI, software
engineering, and developer tooling. Technical precision MUST NOT come at the cost of
comprehensibility.

**Rationale**: A textbook that cannot be understood fails its primary mission. Clear
explanations accelerate learning and reduce misunderstandings that could lead to
implementation errors.

### Reproducibility

All instructions, code, commands, workflows, and deployments MUST be fully reproducible
on any system following the documented steps. Every example MUST be tested and verified.

**Rationale**: Readers should be able to follow along and achieve identical results.
Non-reproducible examples frustrate learners and undermine trust in the material.

### Transparency

All major decisions, assumptions, limitations, and dependencies MUST be explicitly
documented. Hidden complexity or undocumented prerequisites are not acceptable.

**Rationale**: Transparency enables readers to understand not just "how" but "why,"
empowering them to adapt solutions to their specific contexts and make informed
decisions.

### Rigor

Every chapter MUST maintain consistent structure, depth, and technical correctness.
Quality standards apply uniformly across all content.

**Rationale**: Inconsistent quality creates an uneven learning experience. Rigorous
standards ensure every chapter delivers equal value and maintains the textbook's
credibility.

## Key Standards

The following standards MUST be adhered to throughout the project:

- **Official references**: Use official documentation for Docusaurus, Git, GitHub
  Pages, and Spec-Kit Plus. Third-party tutorials or blog posts are insufficient as
  primary sources.
- **Runnable examples**: Include runnable, tested examples for all code and
  configuration snippets. Untested code MUST NOT be published.
- **Consistent formatting**: Use consistent formatting for:
  - Code blocks (with language tags)
  - Architecture diagrams (Mermaid or Excalidraw only)
  - Folder structures (ASCII tree format)
  - Step-by-step workflows (numbered lists with clear prerequisites)
- **Spec-driven workflow**: All features MUST originate from Specs → Tasks → Outputs.
  No ad-hoc development without specification.
- **Review process**: All generated text MUST undergo a review pass for accuracy and
  clarity before publication.

## Constraints

The following constraints define the boundaries of acceptable solutions:

- **Book format**: Docusaurus v3 project with sidebar-structured chapters. No other
  static site generators are permitted.
- **Output deployment**: GitHub Pages (gh-pages branch) with automated build steps.
  Manual deployment processes are not acceptable.
- **Tools used**:
  - Spec-Kit Plus for specs, constitutions, blueprints, and tasks
  - Claude Code for code generation, writing, and refactoring
  - No other AI tools or code generators without explicit approval
- **Diagram format**: Mermaid or Excalidraw only. No proprietary diagram formats or
  image-based diagrams without source files.
- **Content quality**: Avoid filler content — all chapters MUST deliver actionable
  and verifiable information. Vague or general statements without practical value are
  not acceptable.

## Success Criteria

The project is successful when ALL of the following criteria are met:

### Build & Deployment
- The book builds without errors in Docusaurus
- GitHub Pages deploys correctly and is publicly accessible
- All links, images, and interactive elements function correctly

### Technical Quality
- All instructions and code samples execute successfully on a clean system
- All code examples have been tested and verified
- All chapters satisfy:
  - Verified technical accuracy
  - Clear, structured explanations
  - Reproducible examples

### Deliverables
- A complete project folder containing:
  - Specs for all features and chapters
  - This constitution
  - Tasks and implementation records
  - Final deployed book accessible via GitHub Pages

## Governance

### Authority

This constitution is the single source of truth for the project. All work MUST adhere
to these principles, standards, constraints, and success criteria. In case of conflict
between this constitution and any other document, this constitution takes precedence.

### Amendment Procedure

1. **Proposal**: Any amendment MUST be documented with:
   - Rationale for the change
   - Impact analysis on existing work
   - List of affected templates, specs, and artifacts
2. **Review**: Proposed amendments MUST be reviewed for:
   - Consistency with core principles
   - Impact on project success criteria
   - Backward compatibility considerations
3. **Approval**: Amendments require explicit documentation of approval decision
4. **Implementation**: Once approved:
   - Update this constitution with new version number
   - Update `LAST_AMENDED_DATE` to current date
   - Propagate changes to all dependent templates and artifacts
   - Document changes in Sync Impact Report

### Versioning Policy

This constitution follows semantic versioning (MAJOR.MINOR.PATCH):

- **MAJOR**: Backward incompatible changes to governance, principle removals, or
  fundamental redefinitions that invalidate prior work
- **MINOR**: New principles added, sections materially expanded, or new governance
  procedures introduced
- **PATCH**: Clarifications, wording improvements, typo fixes, or non-semantic
  refinements

### Compliance & Review

- All feature specifications MUST include a Constitution Check section verifying
  alignment with these principles
- Implementation plans MUST re-check constitution compliance after design phase
- Regular audits SHOULD be conducted to ensure ongoing adherence
- Violations MUST be documented with justification in Complexity Tracking sections

### Version History

**Version**: 1.1.1
**Ratified**: 2025-12-06
**Last Amended**: 2025-12-11

**Changes in 1.1.1**:
- Verification pass after sp.analyze identified spec-driven workflow compliance issue
- No substantive changes to principles, standards, or governance
- Confirmed all templates and command files correctly reference constitution principles

**Changes in 1.1.0**:
- Enhanced Governance section with formal structure
- Added Amendment Procedure subsection
- Added Versioning Policy subsection
- Added Compliance & Review subsection
- Expanded principle descriptions with explicit rationales
