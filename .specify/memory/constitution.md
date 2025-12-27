<!--
SYNC IMPACT REPORT
==================
Version Change: 1.0.0 → 1.1.0
Constitution Type: MINOR - Phase 6 enhanced with comprehensive UI specifications

Modified/Added Sections:
- ✅ PROJECT IDENTITY (new comprehensive section)
- ✅ AGENT ARCHITECTURE (new - 7 specialized agents defined)
- ✅ CODE QUALITY PRINCIPLES (new - TypeScript, Python, Documentation, Security)
- ✅ TESTING STANDARDS (new - Unit, Component, Integration, E2E)
- ✅ USER EXPERIENCE CONSISTENCY (new - Responsive, Dark Mode, Accessibility, RTL)
- ✅ PERFORMANCE REQUIREMENTS (new - Frontend, Backend, RAG, Translation)
- ✅ ARCHITECTURE DECISIONS (new - Frontend, Backend, Database, AI/ML, Auth, Deployment)
- ✅ DEVELOPMENT PHASES (new - 6 phases with quality gates)
- ✅ ERROR HANDLING POLICY (new - API, User-facing, Graceful degradation)
- ✅ SECURITY PRINCIPLES (new - Secret management, Input sanitization, Auth security)
- ✅ DATA GOVERNANCE (new - Vector store, Relational DB, Embeddings versioning)
- ✅ QUALITY GATES (new - Exit criteria for each phase)
- ✅ GOVERNANCE (expanded - Decision authority, Escalation paths, Skill invocation)
- ✅ NON-FUNCTIONAL REQUIREMENTS (new - Reliability, Observability, Alerting, Cost)
- ✅ OPERATIONAL READINESS (new - Runbooks, Deployment, Feature flags)
- ✅ RISK ANALYSIS (new - Top 3 risks with mitigation)
- ✅ EVALUATION AND VALIDATION (new - Definition of Done)
- ✅ ARCHITECTURAL DECISION RECORDS (new - ADR creation criteria)
- ✅ HUMAN AS TOOL STRATEGY (new - Invocation triggers, Question guidelines)
- ✅ EXECUTION CONTRACT (new - Per request requirements)
- ✅ MINIMUM ACCEPTANCE CRITERIA (new - Per deliverable requirements)
- ✅ PHASE 6 ENHANCED (v1.1.0 - Comprehensive UI Enhancement specification):
  - Zoom-style chatbot panel with docking, resizing, search, unread badges
  - High-quality animations (micro-interactions, page transitions, scroll-based)
  - Navbar icon system with accessibility
  - Home page animations (hero, section reveals, parallax)
  - Core Web Vitals targets (LCP ≤ 2.5s, INP ≤ 200ms, CLS ≤ 0.1)
  - WCAG 2.2 AA compliance with reduced motion support
  - Performance budgets (15KB animation bundle limit, 60fps)
  - Comprehensive testing strategy
  - Risk analysis with kill switches
  - 10 detailed E2E user journeys with Playwright test skeletons:
    1. First-Time Visitor Discovery Flow
    2. Chatbot Panel Full Interaction Cycle
    3. Navbar Navigation with Animations
    4. Chapter Reading with Dark Mode and RTL
    5. Reduced Motion Accessibility
    6. Responsive Breakpoint Transitions
    7. Full Learning Session (Happy Path)
    8. Error Recovery and Graceful Degradation
    9. Keyboard-Only Navigation (Accessibility)
    10. Performance Under Load (Stress Test)

Templates Requiring Updates:
- ✅ plan-template.md - Constitution Check section aligns with new principles
- ✅ spec-template.md - Requirements align with new standards
- ✅ tasks-template.md - Task organization reflects new quality/testing principles
- ⚠️ No command files reference outdated constitution structure

Follow-up Items:
- None - All placeholders filled with concrete project values

Last Generated: 2025-12-10
Last Amended: 2025-12-27 (Phase 6 comprehensive enhancement)
-->

# Physical AI & Humanoid Robotics Textbook - Project Constitution

## PROJECT IDENTITY

**Project Name:** Physical AI & Humanoid Robotics Textbook
**Project Type:** Educational Platform with Integrated AI Features
**Target Audience:** Graduate-level students learning Physical AI, ROS 2, NVIDIA Isaac, and Humanoid Robotics
**Primary Goal:** Deliver an accessible, interactive, multilingual robotics textbook with AI-powered learning assistance

---

## Core Principles

### I. Agent Specialization & Clear Ownership

**Rules:**

- Each agent owns a specific domain with clearly defined skills and responsibilities
- No agent may invoke skills outside its designated skill set
- Cross-domain work requires explicit coordination through the Orchestrator agent
- Skills MUST be invoked through their designated agent owner only

**Rationale:** Prevents skill misuse, ensures domain expertise is properly applied, and maintains clear accountability for deliverables.

**Agent Roster:**

1. **Orchestrator Agent** - Coordination, spec compliance, conflict resolution (no direct skills)
2. **ContentWriter Agent** - Technical authoring (`mdx-writer`)
3. **DocusaurusBuilder Agent** - Frontend infrastructure (`docusaurus-init`, `react-components`, `github-pages-deploy`)
4. **RAGArchitect Agent** - Backend/AI engineering (`qdrant-vectorstore`, `neon-postgres`, `fastapi-backend`, `openai-agents-sdk`)
5. **AuthEngineer Agent** - Authentication (`better-auth-setup`, `user-profiling`, `neon-postgres`)
6. **PersonalizationEngine Agent** - Adaptive content (`content-adapter`, `user-profiling`)
7. **TranslationService Agent** - Urdu translation (`urdu-translator`, `react-components`)

### II. Quality Over Speed

**Rules:**

- Every phase has mandatory quality gates that MUST pass before proceeding
- Test coverage minimum: 80% for all core functionality
- Code review required for all implementations (peer or agent review)
- Documentation MUST be updated before marking work complete
- Performance budgets are non-negotiable (e.g., RAG p95 < 3s)

**Rationale:** Educational platforms require high reliability. Students depend on accurate information and stable systems. Technical debt in educational content compounds learning difficulties.

**Quality Gates per Phase (Summary - see QUALITY GATES section for complete criteria):**

> **Note**: This is a high-level summary only (Tier 2). For authoritative detailed exit criteria, see the QUALITY GATES section below (lines 927-990) which serves as the single source of truth (Tier 1). Per ADR-003, plan phases must match the detailed criteria exactly.

- Phase 1: Lighthouse scores > 85, build success, dark mode functional
- Phase 2: Code examples execute 100%, readability Flesch-Kincaid 12-14
- Phase 3: NDCG@10 > 0.8, p95 latency < 3s, zero hallucinations
- Phase 4A: 100% auth success rate, OWASP Top 10 passed
- Phase 4B: Personalization < 2s, recommendation relevance > 0.75
- Phase 5: Translation < 5s/chapter, 100% technical term preservation
- Phase 6: Core Web Vitals (LCP ≤ 2.5s, INP ≤ 200ms, CLS ≤ 0.1), WCAG 2.2 AA, chatbot panel functional, animations polished

### III. Smallest Viable Change (YAGNI)

**Rules:**

- Implement only what is explicitly required by the current phase
- No premature abstractions or "future-proofing"
- Prefer editing existing code over creating new files
- Avoid backwards-compatibility hacks for unused features
- Delete unused code completely (no commenting out, no `_unused` variables)

**Rationale:** Over-engineering wastes resources and creates maintenance burden. Build incrementally with quality gates between phases ensures we validate before expanding.

**Examples:**

- ✅ Direct database queries → ❌ Repository pattern "for future flexibility"
- ✅ Inline validation → ❌ Complex validation framework for 3 fields
- ✅ Delete unused code → ❌ Keep it "just in case"

### IV. Security by Default

**Rules:**

- No secrets in code (use `.env`, GitHub Secrets for CI/CD)
- All user input sanitized before storage/processing (prevent XSS, SQL injection)
- HTTPS-only for all external API calls
- Passwords hashed with bcrypt (cost factor 12)
- JWT tokens signed with RS256 (not HS256)
- Rate limiting on all authentication endpoints (10 login attempts per 15 min per IP)
- OWASP Top 10 compliance required before Phase 4A exit

**Rationale:** Educational platforms collect user data (profiles, learning progress). Security breaches erode trust and violate data privacy obligations (GDPR).

### V. Observability & Measurability

**Rules:**

- Structured JSON logging for all backend operations
- Correlation IDs for request tracing (API errors, RAG queries)
- Log levels: DEBUG, INFO, WARN, ERROR, CRITICAL
- Metrics tracked: API latency (p50/p95/p99), error rates, RAG relevance scores, user engagement
- All success criteria MUST be measurable (no vague "high quality" statements)

**Rationale:** Cannot improve what is not measured. Observability enables debugging, performance optimization, and data-driven decisions.

**Examples:**

- ✅ "RAG response time p95 < 3s" → ❌ "Fast RAG responses"
- ✅ "80% test coverage" → ❌ "Well-tested code"

### VI. Accessibility & Inclusivity (WCAG 2.2 AA)

**Rules:**

- All images MUST have alt text
- Keyboard navigation functional for all interactive elements
- Focus indicators visible and styled
- Color contrast ratios: 4.5:1 (text), 3:1 (UI components)
- Screen reader tested (NVDA/JAWS)
- No color-only information conveyance
- RTL support for Urdu (text direction, layout mirroring, icon flipping)
- Touch targets ≥ 44x44px on mobile

**Rationale:** Education should be accessible to all learners regardless of ability. Accessibility is not optional for educational platforms.

### VII. Free Tier Sustainability

**Rules:**

- Respect all free tier limits (Qdrant 1GB, Neon 0.5GB, GitHub Pages 100GB/month)
- Monitor usage proactively (alert at 80% of limits)
- Implement rate limiting to protect quotas (10 queries/hour free users, 50/hour authenticated)
- Cache aggressively (frequent queries, translated chapters)
- Document cost budgets and unit economics (target: < $0.01 per user per month)

**Rationale:** Project sustainability depends on staying within free tier constraints. Cost overruns force platform shutdown or paid model that excludes learners.

### VIII. Prompt History Records (PHR) for All Work

**Rules:**

- Create PHR after EVERY user prompt that results in implementation, planning, debugging, or spec work
- Route PHRs to appropriate subdirectory:
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/`
  - `general` → `history/prompts/general/`
- Fill ALL template fields (no unresolved placeholders)
- Embed full PROMPT_TEXT (verbatim, not truncated)
- Include concise RESPONSE_TEXT
- Validate: ID, path, stage, title printed; path matches route

**Rationale:** PHRs provide traceability, learning history, and audit trail for all agent work. They enable retrospectives and debugging.

**Exception:** Skip PHR only for `/sp.phr` itself (avoid infinite recursion).

### IX. Architectural Decision Records (ADR) for Significant Decisions

**Rules:**

- Suggest ADR creation when ALL three criteria met:
  1. **Impact:** Long-term consequences? (framework, data model, API, security, platform)
  2. **Alternatives:** Multiple viable options considered?
  3. **Scope:** Cross-cutting and influences system design?
- NEVER auto-create ADRs; always wait for user consent
- ADR format: Context, Decision, Alternatives Considered, Consequences, Status
- Store in `history/adr/NNNN-<slug>.md`
- Group related decisions (e.g., authentication stack) into one ADR when appropriate

**Rationale:** ADRs capture the "why" behind architectural choices, enabling future maintainers to understand tradeoffs and constraints.

**Suggestion Format:**

```
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

### X. Human as Tool (Explicit Escalation)

**Rules:**

- Agent MUST escalate to user when:
  1. **Ambiguous Requirements:** User intent unclear → Ask 2-3 targeted questions
  2. **Unforeseen Dependencies:** Dependencies not in spec → Surface and ask for prioritization
  3. **Architectural Uncertainty:** Multiple valid approaches → Present options with tradeoffs
  4. **Completion Checkpoint:** Major milestone reached → Summarize work, confirm next steps
- Provide context and options, not just "What should I do?"
- State recommendation if you have one

**Rationale:** Agents are not omniscient. User judgment is required for ambiguous decisions. Escalation prevents wasted work on wrong assumptions.

---

## CODE QUALITY PRINCIPLES

### TypeScript Standards

1. **Strict Mode Enabled:** No `any` types except in explicit migration scenarios
2. **Type Coverage:** 100% for all public APIs
3. **JSDoc Required:** All exported functions, components, and types
4. **Naming Conventions:**
   - Components: PascalCase
   - Functions: camelCase
   - Constants: UPPER_SNAKE_CASE
   - Files: kebab-case

### Python Standards

1. **PEP 8 Compliance:** Enforced via `black` and `flake8`
2. **Type Hints:** Required for all function signatures (mypy strict mode)
3. **Docstrings:** Google-style for all public functions and classes
4. **Async First:** Use `async/await` for all I/O operations

### Documentation Requirements

1. **README:** Each major directory has README.md explaining structure
2. **API Docs:** OpenAPI/Swagger for all FastAPI endpoints
3. **Component Docs:** Storybook or equivalent for React components
4. **Architecture Docs:** ADRs for all significant decisions

### Security Standards

1. **No Secrets in Code:** Use `.env` files, never commit
2. **Input Validation:** Sanitize all user input before storage/processing
3. **Dependency Scanning:** Run `npm audit` and `pip-audit` weekly
4. **HTTPS Only:** All external API calls use HTTPS

---

## TESTING STANDARDS

### Unit Testing

**Minimum Coverage:** 80% for all core functionality
**Framework:** Jest (TypeScript), Pytest (Python)

**Requirements:**

- All utility functions tested
- All business logic tested
- Edge cases and error paths covered

### Component Testing

**Framework:** React Testing Library

**Requirements:**

- All interactive components tested
- User interactions simulated (click, input, navigation)
- Accessibility checks included (axe-core)
- Snapshot tests for visual regression

### Integration Testing

**RAG Pipeline:**

- Test end-to-end query flow (input → embedding → search → generation)
- Validate relevance scores on 50-question test set
- Measure latency for representative queries
- **Pass Criteria:** NDCG@10 > 0.8, p95 latency < 3s

**Authentication Flow:**

- Test email signup, login, logout
- Test Google OAuth flow
- Test token refresh and expiration
- **Pass Criteria:** 100% success on test scenarios

### Translation Testing

**Validation:**

- 100% technical term preservation (automated check against term list)
- Grammar correctness (native speaker review sample)
- RTL rendering (visual regression tests)
- **Pass Criteria:** Zero layout breaks, term preservation 100%

### End-to-End Testing

**Framework:** Playwright

**Critical User Journeys:**

1. New user signup → browse chapters → ask RAG question → get answer
2. Existing user login → personalized recommendations → bookmark chapter
3. User switches to Urdu → RTL layout renders → translation displays correctly

**Pass Criteria:** All journeys complete without errors

---

## USER EXPERIENCE CONSISTENCY

### Responsive Design

**Breakpoints:**

- Mobile: 320px - 767px
- Tablet: 768px - 1023px
- Desktop: 1024px+

**Requirements:**

- Touch targets ≥ 44x44px (mobile)
- Readable text without zoom (16px base font size)
- No horizontal scrolling at any breakpoint

### Dark Mode

**Implementation:**

- System preference detection on first visit
- Manual toggle persists in localStorage
- All components support dark mode
- Color contrast ratios meet WCAG AA (4.5:1 text, 3:1 UI)

### Accessibility (WCAG 2.1 AA)

**Requirements:**

- All images have alt text
- Keyboard navigation works for all interactive elements
- Focus indicators visible and styled
- Screen reader tested (NVDA/JAWS)
- No color-only information conveyance

### Loading States

**Guidelines:**

- Skeleton screens for content loading
- Spinners for async operations < 3s
- Progress bars for operations > 3s
- Error boundaries catch and display friendly errors

### Error Handling

**User-Facing Errors:**

- Clear, actionable error messages
- i18n-compatible (English + Urdu)
- Retry options where applicable
- Contact support link for critical errors

**Backend Errors:**

- Logged with correlation IDs
- Structured logging (JSON format)
- Severity levels (INFO, WARN, ERROR, CRITICAL)

### RTL Support (Urdu)

**Requirements:**

- Text direction switches to RTL
- Layout mirrors (sidebar, navigation)
- Icons and buttons flip appropriately
- No hardcoded LTR assumptions in CSS

---

## PERFORMANCE REQUIREMENTS

### Frontend Performance

**Metrics:**

- Initial page load (First Contentful Paint): < 1.5s
- Time to Interactive: < 3s
- Largest Contentful Paint: < 2.5s
- Cumulative Layout Shift: < 0.1

**Optimization:**

- Code splitting by route
- Image lazy loading
- Font subsetting
- CDN for static assets

### Backend Performance

**API Latency (p95):**

- GET endpoints: < 200ms
- POST endpoints: < 500ms
- RAG query: < 3s (end-to-end)
- Translation: < 5s per chapter

**Optimization:**

- Database query indexing
- Redis caching for frequent queries
- Connection pooling
- Async operations for I/O

### RAG Chatbot Performance

**Detailed Budget:**

- Embed user query (OpenAI API): < 500ms
- Search Qdrant (top 10 chunks): < 1s
- Generate answer (OpenAI Agents SDK): < 1.5s
- **Total:** < 3s (p95)

**Optimization:**

- Batch embedding requests where possible
- Qdrant HNSW index optimization (M=16, ef_construct=100)
- Prompt caching for repeated queries
- Rate limiting to protect quotas

### Translation Performance

**Target:** < 5s per chapter

**Optimization:**

- Stream translation (yield chunks as available)
- Cache translated chapters
- Background job for bulk translation

### Lighthouse Targets

- Performance: > 90
- Accessibility: > 90
- Best Practices: > 90
- SEO: > 90

---

## ARCHITECTURE DECISIONS

### Frontend Architecture

**Framework:** Docusaurus v3 (React-based static site generator)
**Rationale:** Built for documentation, excellent DX, plugin ecosystem
**Language:** TypeScript (strict mode)
**Styling:** CSS Modules + Docusaurus theme tokens
**State Management:** React Context for global state (user, theme)

### Backend Architecture

**Framework:** FastAPI (Python 3.11+)
**Rationale:** High performance, async support, auto OpenAPI docs
**API Design:** RESTful + OpenAPI 3.1
**Deployment:** Containerized (Docker)

### Database Architecture

**Relational Data:** Neon Serverless Postgres (Free Tier)

**Schema:**

- `users` table (auth, profile)
- `chat_history` table (user queries, bot responses)
- `user_preferences` table (learning level, bookmarks)

**Vector Data:** Qdrant Cloud (Free Tier, 1GB)

**Collections:**

- `textbook_chunks` (chapter embeddings, metadata)

### AI/ML Architecture

**Embedding Model:** OpenAI `text-embedding-3-small` (1536 dimensions)
**LLM:** OpenAI GPT-4o-mini via Agents SDK
**Rationale:** Cost-effective, good performance for educational content

### Authentication Architecture

**Library:** Better Auth (email + Google OAuth)
**Session Storage:** Neon Postgres
**Token Type:** JWT (RS256 signing)
**Expiry:** 24h access token, 30d refresh token

### Deployment Architecture

**Frontend:** GitHub Pages (free static hosting)
**Backend:** Railway / Render free tier (containerized FastAPI)
**CI/CD:** GitHub Actions (build, test, deploy)

### Free Tier Constraints

**Respected Limits:**

- Qdrant: 1GB storage (~100k chunks)
- Neon: 0.5GB storage, 100 compute hours/month
- OpenAI: Rate limits per API key tier
- GitHub Pages: 100GB bandwidth/month

---

## DEVELOPMENT PHASES

### Phase 1: Book Infrastructure (Owner: DocusaurusBuilder)

**Deliverables:**

- Docusaurus v3 initialized with TypeScript
- Custom theme with dark mode
- Responsive layout (mobile, tablet, desktop)
- GitHub Pages deployment configured

**Quality Gate:**

- Site builds without errors
- Deploys to GitHub Pages successfully
- Lighthouse scores all > 85
- Dark mode toggle functional

---

### Phase 2: Content Creation (Owner: ContentWriter)

**Deliverables:**

- 10 sample chapters (MDX format)
- Code examples tested and runnable
- Learning objectives per chapter
- Exercises with solutions

**Quality Gate:**

- All chapters render correctly in Docusaurus
- Code examples execute without errors (tested in CI)
- Peer review completed by domain expert
- Readability appropriate for target audience

---

### Phase 3: RAG Chatbot Core (Owner: RAGArchitect)

**Deliverables:**

- Vector database populated with chapter chunks
- FastAPI backend with `/query` endpoint
- OpenAI Agents SDK integration
- Basic chatbot UI (simplified)

**Quality Gate:**

- Retrieval NDCG@10 > 0.8 on test set
- Response time p95 < 3s
- No hallucinations on 50-question test set
- API documented with OpenAPI

---

### Phase 4A: Authentication (Owner: AuthEngineer)

**Deliverables:**

- Better Auth configured (email + Google)
- User profile schema in Neon Postgres
- Session management functional
- Protected routes implemented

**Quality Gate:**

- Email signup, login, logout work (100% success)
- Google OAuth flow functional
- Tokens expire and refresh correctly
- Passes OWASP Top 10 security checklist

---

### Phase 4B: Personalization (Owner: PersonalizationEngine)

**Deliverables:**

- User skill level classification
- Personalized chapter recommendations
- Learning progress tracking
- Adaptive content depth

**Quality Gate:**

- Personalization response time < 2s
- Recommendation relevance > 0.75
- User feedback survey shows > 70% satisfaction

---

### Phase 5: Translation

**Purpose:** Deliver multilingual learning experience with technical accuracy and proper RTL support for Urdu-speaking students.

**Owner:** TranslationService Agent

**Skills Used:** `urdu-translator`, `react-components`

---

**Deliverables:**

- Urdu translation for all chapters with technical term preservation

- RTL (Right-to-Left) layout components and styles

- Language toggle UI with persistent user preference

- Technical term glossary (bilingual: English-Urdu)

- Translation quality validation tooling

- Cached translation storage for performance

---

**Quality Gate:**

- Translation generation time < 5s per chapter (end-to-end)

- 100% technical term preservation (automated validation against glossary)

- RTL layout renders without breaks (visual regression testing on 3 viewports)

- Native speaker review passes for 3 sample chapters (grammar, technical accuracy, readability)

- Language toggle persists across sessions (localStorage validation)

- No layout shift when switching languages (CLS < 0.1)

---

**Implementation Requirements:**

**1. Translation Service Architecture:**

- Use `urdu-translator` skill for content translation

- Preserve technical terms: ROS, Python, NVIDIA Isaac, Gazebo, URDF, etc.

- Maintain code block syntax and variable names unchanged

- Support MDX frontmatter and metadata translation

- Cache translated chapters to reduce API calls and cost

**2. RTL Layout Implementation:**

- CSS logical properties: Use `margin-inline-start` instead of `margin-left`

- Directional icons: Flip navigation arrows, breadcrumbs, and UI icons

- Text alignment: Right-align for Urdu, left-align for English

- Layout mirroring: Sidebar, navigation, and grid layouts flip for RTL

- No hardcoded left/right values in component styles

- Bidirectional (BiDi) text handling for mixed English-Urdu content

**3. Language Toggle UI:**

- Accessible toggle component (WCAG 2.1 AA compliant)

- Keyboard navigable (Tab, Enter, Space support)

- ARIA labels: `aria-label="Switch to Urdu"` / `aria-label="Switch to English"`

- Visual indicator of current language (not color-only)

- Mobile-friendly touch target (≥ 44x44px)

- Persist language preference in localStorage

- Sync language preference with user profile (if authenticated)

**4. Technical Term Glossary:**

- Bilingual glossary file: English term → Urdu transliteration + definition

- Automated validation: Flag any translated term that differs from glossary

- Hover tooltips: Show English term on hover for Urdu technical terms

- Glossary page: Searchable, filterable, exportable reference

- Version control: Track glossary updates in git

**5. Translation Quality Validation:**

- **Automated checks:**
  - Technical term preservation: 100% match against glossary

  - Code block integrity: No translation inside code fences

  - MDX syntax preservation: No broken JSX components

  - Link integrity: All internal/external links functional

- **Manual review:**
  - Native speaker review 3 sample chapters

  - Grammar correctness: Flesch-Kincaid equivalent for Urdu readability

  - Cultural appropriateness: Educational tone suitable for graduate students

- **Visual regression:**
  - Screenshot comparison: English vs Urdu layouts (3 viewports: 320px, 768px, 1024px)

  - No text overflow, no layout breaks, no overlapping elements

  - Proper font rendering: Use Noto Nastaliq Urdu or equivalent

---

**Performance Budget:**

- Translation generation: < 3s per chapter (API call to urdu-translator)

- Translation caching: First load 3s, subsequent loads < 500ms (from cache)

- Language toggle response: < 200ms (instant UI switch)

- Total time to switch language: < 1s (including font loading)

- Font loading: Use `font-display: swap` to prevent FOIT (Flash of Invisible Text)

---

**Accessibility Requirements (WCAG 2.1 AA):**

- Screen reader announces language switch: `<html lang="ur">` for Urdu

- RTL reading order matches visual order

- Focus indicators visible in both LTR and RTL modes

- All interactive elements keyboard accessible

- Color contrast ratios maintained: 4.5:1 (text), 3:1 (UI components)

- No reliance on directional cues alone ("click the button on the right")

---

**Security Considerations:**

- **Translation API Security:**
  - No API keys in client-side code (use environment variables)

  - Rate limiting: 10 translation requests per minute per user (prevent abuse)

  - Input sanitization: Prevent XSS through translated content

  - Validate translation output: Reject responses with script tags or malicious content

- **Content Integrity:**
  - Hash original content: Detect tampering or unauthorized modifications

  - Store translations in secure database (Neon Postgres, encrypted at rest)

  - Audit trail: Log who translated what and when

---

**Error Handling & Graceful Degradation:**

- **Translation service unavailable:**
  - Fallback: Display English version with banner "Translation temporarily unavailable"

  - Cache previous translations: Serve stale cache if API fails

  - Retry logic: 3 attempts with exponential backoff (2s, 4s, 8s)

- **Partial translation failure:**
  - Show translated sections + English for untranslated sections

  - Highlight untranslated sections with visual indicator

- **RTL rendering issues:**
  - Fallback to LTR with Urdu text (readable but not optimal)

  - Log error for developer investigation

- **User-facing error messages:**
  - Clear, actionable: "Translation failed. Showing English version. [Retry]"

  - Bilingual error messages (English + Urdu) where applicable

---

**Testing Strategy:**

**1. Unit Tests:**

- Technical term preservation logic (100% glossary match)

- RTL CSS class application (verify direction changes)

- Language toggle state management (localStorage persistence)

**2. Component Tests:**

- Language toggle component (click, keyboard interaction)

- RTL layout components (text direction, icon flipping)

- Glossary tooltip component (hover, click, accessibility)

**3. Integration Tests:**

- End-to-end translation flow: Request → API → Cache → Render

- Language switch flow: Toggle → Fetch translation → Update UI → Persist preference

- Bilingual glossary search and filtering

**4. Visual Regression Tests:**

- Percy or Chromatic: Screenshot comparison for 10 chapters

- Test 3 viewports: Mobile (320px), Tablet (768px), Desktop (1024px)

- Compare English vs Urdu layouts: No layout breaks, proper alignment

**5. Manual Testing:**

- Native speaker review: 3 sample chapters (grammar, accuracy, readability)

- Cross-browser testing: Chrome, Firefox, Safari, Edge

- Cross-device testing: iOS, Android, Desktop

- Screen reader testing: NVDA (Windows), VoiceOver (macOS/iOS)

---

**Free Tier Sustainability:**

- **Translation API Costs:**
  - Monitor API usage: Alert at 80% of monthly quota

  - Aggressive caching: Cache all translations indefinitely

  - Background translation: Pre-translate all chapters during deployment (not on-demand)

  - Rate limiting: 10 translations per user per day (free tier), unlimited (authenticated)

- **Storage Optimization:**
  - Store translations in Neon Postgres (within 0.5GB limit)

  - Compress translation payloads (gzip compression)

  - Deduplicate common phrases and technical terms

---

**Success Criteria:**

- All 10 chapters translated to Urdu (100% coverage)

- Translation time < 5s per chapter (measured in production)

- Technical term preservation 100% (automated validation passes)

- RTL layout renders correctly (visual regression tests pass on 3 viewports)

- Native speaker review passes for 3 sample chapters (score ≥ 80% on quality rubric)

- Language toggle functional and accessible (WCAG AA audit passes)

- Zero layout shift when switching languages (CLS < 0.1 measured)

- PHR created documenting translation implementation approach

---

**Does NOT Include (Out of Scope for Phase 5):**

- Machine translation for languages other than Urdu

- Audio/video content translation

- Real-time translation chat interface

- User-contributed translations or crowdsourcing

- Translation memory or CAT (Computer-Assisted Translation) tools

---

**Follow-Up Items (Post Phase 5):**

- Monitor user feedback on translation quality

- Iterate on glossary based on student questions

- A/B test different Urdu fonts for readability

- Add keyboard shortcuts for language switching (e.g., Alt+L)

- Implement translation analytics (which chapters most viewed in Urdu)

---

**Risks & Mitigation:**

**Risk 1: Translation Quality Issues**

- **Impact:** Medium (poor UX for Urdu users, learning comprehension affected)

- **Likelihood:** Medium

- **Mitigation:**
  - Mandatory native speaker review before deployment

  - Technical term glossary enforced (100% automation)

  - User feedback mechanism (report translation errors)

  - Iterative improvement based on feedback

- **Kill Switch:** Disable Urdu toggle, show English-only version

**Risk 2: RTL Layout Breaks**

- **Impact:** High (unusable UI for Urdu users)

- **Likelihood:** Low (if visual regression tests pass)

- **Mitigation:**
  - Comprehensive visual regression testing (Percy/Chromatic)

  - CSS logical properties from the start (no left/right hardcoding)

  - Cross-browser and cross-device testing

  - Fallback to LTR + Urdu text if critical RTL failure

- **Kill Switch:** Fallback to English-only mode

**Risk 3: Translation API Cost Overrun**

- **Impact:** Medium (budget exhaustion)

- **Likelihood:** Low (aggressive caching prevents repeated calls)

- **Mitigation:**
  - Pre-translate all chapters during deployment (one-time cost)

  - Cache translations indefinitely (no re-translation)

  - Monitor API spend daily, alert at threshold

  - Rate limiting per user (10 requests/day free tier)

- **Kill Switch:** Disable on-demand translation, serve cached only

---

**Constitution Alignment:**

- ✅ Accessibility & Inclusivity (Principle VI): Full RTL support, WCAG 2.1 AA compliance

- ✅ Free Tier Sustainability (Principle VII): Aggressive caching, rate limiting, monitoring

- ✅ Security by Default (Principle IV): API key protection, input sanitization, audit trail

- ✅ Quality Over Speed (Principle II): Native speaker review, automated validation, visual regression testing

- ✅ Smallest Viable Change (Principle III): Only Urdu translation (not other languages), no advanced CAT tools

- ✅ Observability & Measurability (Principle V): Translation time measured, term preservation tracked, API usage monitored

---

---

## ERROR HANDLING POLICY

### API Error Handling

**Requirements:**

- All external API calls have timeout (default 30s)
- Retry logic with exponential backoff (3 attempts)
- Circuit breaker for repeated failures
- Correlation IDs for request tracing

**Error Response Format:**

```json
{
  "error": {
    "code": "EMBEDDING_TIMEOUT",
    "message": "Failed to generate embedding",
    "correlation_id": "abc-123-def",
    "timestamp": "2025-12-10T12:34:56Z"
  }
}
```

### User-Facing Errors

**Guidelines:**

- Clear, non-technical language
- Actionable next steps
- Avoid exposing internal details
- i18n-compatible (English + Urdu)

**Examples:**

- ✅ "Unable to find relevant answers. Try rephrasing your question."
- ❌ "Vector search returned empty results from Qdrant."

### Graceful Degradation

**Strategies:**

- RAG failure → Show static FAQ content
- Translation unavailable → English fallback
- Personalization error → Default recommendations
- Auth service down → Allow guest browsing (no chat)

---

## SECURITY PRINCIPLES

### Secret Management

**Rules:**

- No API keys, passwords, or tokens in code
- Use `.env` files for local development
- Use GitHub Secrets for CI/CD
- Rotate secrets quarterly (document in ADR)

**Environment Variables:**

```
OPENAI_API_KEY=sk-...
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgres://...
GITHUB_CLIENT_SECRET=...
```

### Input Sanitization

**Requirements:**

- Sanitize all user input before storage
- Prevent SQL injection (use parameterized queries)
- Prevent XSS (escape HTML in user content)
- Validate file uploads (type, size)

### Authentication Security

**Requirements:**

- Passwords hashed with bcrypt (cost factor 12)
- JWT signed with RS256 (not HS256)
- CSRF tokens for state-changing requests
- Rate limiting on auth endpoints

**Rate Limits:**

- Login attempts: 10 per 15 minutes per IP
- Signup: 5 per hour per IP
- Password reset: 3 per hour per email

### Data Privacy

**Principles:**

- Collect minimum necessary user data
- Store chat history for max 30 days (user-deletable)
- No PII in vector database
- GDPR-compliant data export/deletion

---

## DATA GOVERNANCE

### Vector Store (Qdrant)

**Contents:** Textbook chapter chunks + metadata

**Schema:**

```json
{
  "id": "chapter-3-section-2-chunk-5",
  "vector": [0.123, ...],
  "payload": {
    "chapter": "3",
    "section": "2",
    "title": "ROS 2 Publishers",
    "content": "...",
    "tags": ["ros2", "publisher"]
  }
}
```

**Retention:** Indefinite (reference material)
**PII:** None allowed

### Relational Database (Neon Postgres)

**Tables:**

1. `users` - Auth info, encrypted email
2. `user_profiles` - Learning preferences, skill level
3. `chat_history` - User queries + bot responses (30-day retention)
4. `user_bookmarks` - Saved chapters

**Encryption:** At rest (Neon default), in transit (TLS 1.3)
**Backup:** Neon automatic daily backups (7-day retention)

### Embeddings Versioning

**Strategy:**

- Store embedding model name + version in metadata
- On model upgrade, re-embed all content
- Keep old embeddings for rollback (7 days)

---

## QUALITY GATES

### Phase 1 Exit Criteria (Book Infrastructure)

- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] Site deploys to GitHub Pages without errors
- [ ] Lighthouse Performance > 85
- [ ] Lighthouse Accessibility > 90
- [ ] Dark mode toggle functional
- [ ] Responsive on mobile (320px), tablet (768px), desktop (1024px)
- [ ] PHR created documenting infrastructure decisions

### Phase 2 Exit Criteria (Content Creation)

- [ ] 10 chapters authored in MDX format
- [ ] All code examples execute without errors (CI test)
- [ ] Learning objectives defined per chapter
- [ ] Peer review completed by domain expert
- [ ] Readability Flesch-Kincaid score 12-14
- [ ] PHR created summarizing content milestones

### Phase 3 Exit Criteria (RAG Chatbot Core)

- [ ] Vector database has 1000+ chunks
- [ ] `/query` API endpoint functional and documented
- [ ] NDCG@10 > 0.8 on 50-question test set
- [ ] Response time p95 < 3s (measured in CI)
- [ ] Zero hallucinations on test set
- [ ] ADR created for RAG architecture decisions
- [ ] PHR created documenting RAG implementation

### Phase 4A Exit Criteria (Authentication)

- [ ] Email signup, login, logout work (100% success on test scenarios)
- [ ] Google OAuth functional
- [ ] JWT tokens expire after 24h and refresh correctly
- [ ] OWASP Top 10 checklist passed (security audit)
- [ ] Rate limiting functional (tested with load tool)
- [ ] PHR created documenting auth implementation

### Phase 4B Exit Criteria (Personalization)

- [ ] User skill level classification functional
- [ ] Personalized recommendations display correctly
- [ ] Response time < 2s (measured)
- [ ] Recommendation relevance > 0.75 (A/B test)
- [ ] PHR created documenting personalization logic

### Phase 5 Exit Criteria (Translation)

- [ ] All 10 chapters translated to Urdu
- [ ] Translation time < 5s per chapter (measured)
- [ ] 100% technical term preservation (automated check)
- [ ] RTL layout renders correctly (visual regression tests)
- [ ] Native speaker review passes for 3 sample chapters
- [ ] PHR created documenting translation approach

### Phase 6: Enhancing the UI

**Purpose:** Deliver a polished, delightful user experience with high-quality animations, an upgraded chatbot interface, improved navigation, and enhanced visual feedback across all interactions.

**Owner:** DocusaurusBuilder Agent + UI Enhancement Specialist

**Skills Used:** `react-components`, `docusaurus-init`, animation libraries (Framer Motion or CSS-based)

---

**Deliverables:**

1. **High-Quality Animations Across the App**
   - Micro-interactions (button presses, input focus, form validation feedback)
   - Page transitions (route changes, tab switches)
   - Hover and press states with visual feedback
   - Loading state animations (skeleton screens, spinners, progress indicators)
   - Success/error state animations (checkmarks, shake effects)

2. **Upgraded Chatbot Experience ("Zoom-Style Chat Panel")**
   - Docked right-side panel with fixed positioning
   - Collapsible/minimize functionality with smooth transitions
   - Resizable width (drag handle, min/max constraints)
   - Unread message badge with count indicator
   - Quick jump to latest message button
   - Search functionality within chat history
   - Smooth open/close transitions (no layout jank)
   - Beautiful loading animation during content generation
   - Message bubble animations (fade-in, slide-up)
   - Typing indicator with animated dots

3. **Navbar Icon Set Enhancement**
   - Clear, consistent icons (use established icon library: Lucide, Heroicons, or Phosphor)
   - Consistent sizing (24x24px default, 20x20px compact)
   - Accessible labels and tooltips for all icons
   - Hover states with subtle scale/opacity transitions
   - Active state indicators for current section
   - Mobile hamburger menu with smooth drawer animation

4. **Index (Home) Page Animations**
   - Hero section entrance animation (fade-in + slide-up, staggered elements)
   - Section reveal animations on scroll (intersection observer based)
   - Subtle parallax effects for visual depth (performance-optimized)
   - Feature cards with hover lift effects
   - Call-to-action button animations (pulse, glow effects)
   - Statistics/counter animations (number count-up effect)

---

**Quality Gate:**

**Performance Targets (Core Web Vitals):**
- Largest Contentful Paint (LCP): ≤ 2.5s
- Interaction to Next Paint (INP): ≤ 200ms
- Cumulative Layout Shift (CLS): ≤ 0.1

**Accessibility (WCAG 2.2 AA Compliance):**
- All animations respect `prefers-reduced-motion` user preference
- Keyboard navigation works for all interactive elements
- Focus indicators visible and styled (3:1 contrast ratio)
- No focus obstruction (overlays don't trap focus unexpectedly)
- Animated content doesn't auto-play for more than 5 seconds
- Readable color contrast maintained (4.5:1 text, 3:1 UI components)
- Screen reader announces chatbot state changes (opened, closed, new messages)

**Reduced Motion Support:**
- Detect `prefers-reduced-motion: reduce` media query
- Provide alternative static states for all animations
- Critical animations (loading indicators) simplified but not removed
- Transitions reduced to opacity-only (no movement)

**Responsive UX:**
- Navbar works cleanly on mobile, tablet, and desktop
- Chatbot panel adapts to screen size (full-screen on mobile, docked on desktop)
- No horizontal overflow at any breakpoint (320px - 2560px)
- Touch targets ≥ 44x44px on mobile
- No broken layouts during animation playback

---

**Implementation Requirements:**

**1. Animation Library Strategy:**

- **Preferred:** CSS animations and transitions (native, zero bundle cost)
- **When CSS insufficient:** Framer Motion (React-optimized, tree-shakeable)
- **Avoid:** GSAP, Anime.js, or similar heavy libraries unless justified
- **Bundle budget:** Animation libraries must not add > 15KB gzipped

**2. Animation Design Principles:**

- **Duration:** 150-300ms for micro-interactions, 300-500ms for page transitions
- **Easing:** Use natural easing curves (`ease-out` for entrances, `ease-in` for exits)
- **Stagger:** Delay child elements by 50-100ms for lists/grids
- **Purpose:** Every animation must serve UX purpose (feedback, orientation, delight)
- **Subtlety:** Prefer subtle over flashy (educational platform, not marketing site)

**3. Chatbot Panel Architecture:**

```typescript
interface ChatPanelProps {
  isOpen: boolean;
  isMinimized: boolean;
  width: number; // 320-600px range
  unreadCount: number;
  onToggle: () => void;
  onMinimize: () => void;
  onResize: (width: number) => void;
}

// State persistence
- Panel open/closed state: localStorage
- Panel width preference: localStorage
- Minimize state: sessionStorage (reset on refresh)
```

**4. Chat Panel Features:**

- **Docked Position:** Fixed to right viewport edge, below navbar
- **Z-index:** Below modals, above page content (z-index: 100)
- **Resize Handle:** Left edge, cursor: `ew-resize`, visual indicator on hover
- **Width Constraints:** Min 320px, Max 600px, Default 400px
- **Minimize Mode:** Collapse to icon-only bar (64px height)
- **Unread Badge:** Red dot with count, positioned top-right of toggle button
- **Jump to Latest:** Floating button appears when scrolled up > 200px
- **Search:** Input field at top, highlights matching messages, keyboard shortcut (Ctrl+F in panel)

**5. Loading Animation Requirements:**

- **Skeleton Screens:** For content areas taking > 200ms to load
- **Chatbot Typing Indicator:** 3-dot animation during AI response generation
- **Progress Bar:** For operations > 3s (translation, file uploads)
- **Spinner:** For quick async operations (< 3s)
- **Content Shimmer:** Gradient animation on skeleton elements

**6. Scroll-Based Animations:**

- Use `IntersectionObserver` API (not scroll event listeners)
- Threshold: Trigger when 20% of element visible
- Once-only: Most animations play once (no repeat on scroll up)
- Performance: Use `will-change` sparingly, prefer `transform` and `opacity`

**7. Icon System:**

- **Library:** Lucide React (tree-shakeable, consistent design)
- **Sizes:** 16px (inline), 20px (compact UI), 24px (standard), 32px (hero)
- **Stroke Width:** 1.5px (default), 2px (bold/emphasis)
- **Colors:** Inherit from parent (no hardcoded colors)
- **Accessibility:** `aria-hidden="true"` for decorative, `aria-label` for functional

---

**Performance Budget:**

- **Animation Frame Rate:** Maintain 60fps during all animations
- **JavaScript Execution:** < 50ms for animation triggers
- **Layout Thrashing:** Zero forced synchronous layouts
- **Paint Operations:** Minimize repaints, prefer compositor-only animations
- **Bundle Impact:** Animation code ≤ 15KB gzipped total
- **First Input Delay:** < 100ms (animations don't block input)
- **Time to Interactive:** No regression from Phase 5 baseline

**Animation Performance Checklist:**
- Use `transform` and `opacity` only (GPU accelerated)
- Avoid animating `width`, `height`, `top`, `left`, `margin`, `padding`
- Use `will-change` only during active animation (remove after)
- Debounce scroll handlers (if any) to 16ms minimum
- Use `requestAnimationFrame` for JavaScript animations

---

**Accessibility Requirements (WCAG 2.2 AA):**

**1. Reduced Motion:**

```css
@media (prefers-reduced-motion: reduce) {
  *, *::before, *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

- Critical animations (loading spinners) use opacity-only alternative
- Page transitions become instant fade (no slide)
- Scroll animations disabled, elements render immediately

**2. Keyboard Navigation:**

- All animated interactive elements focusable
- Focus order follows visual order (no disruption from animations)
- Escape key closes chatbot panel
- Arrow keys navigate within chatbot messages
- Tab cycles through chatbot input, send button, minimize, close

**3. Screen Reader Announcements:**

- Chatbot open: "Chat panel opened"
- Chatbot close: "Chat panel closed"
- New message: "New message from assistant" (live region)
- Unread count: "3 unread messages" (badge has aria-label)
- Loading state: "Generating response, please wait"

**4. Focus Management:**

- Opening chatbot moves focus to input field
- Closing chatbot returns focus to toggle button
- Focus trap within open modal overlays
- Focus visible indicator on all interactive elements

---

**Error Handling & Graceful Degradation:**

**1. Animation Failures:**

- CSS animations fail gracefully (element simply appears)
- JavaScript animation errors caught, logged, element rendered without animation
- No blank screens or broken layouts from animation errors

**2. Chatbot Panel Failures:**

- Panel fails to open: Show error toast, fallback to inline chat (if available)
- Resize breaks: Reset to default width, log error
- State persistence fails: Use defaults, warn in console

**3. Icon Loading Failures:**

- Missing icons: Fall back to text label
- Icon library fails to load: Critical icons have inline SVG fallbacks

**4. Performance Degradation:**

- Detect low frame rate (< 30fps sustained): Disable non-essential animations
- Detect low memory: Reduce animation complexity
- Detect slow network: Skip loading animations, show instant states

---

**Testing Strategy:**

**1. Unit Tests:**

- Animation hook logic (useReducedMotion, useScrollAnimation)
- Chatbot panel state management (open, close, minimize, resize)
- Icon rendering with various props
- Animation utility functions (easing, timing calculations)

**2. Component Tests:**

- Chatbot panel interactions (open, close, minimize, resize, scroll)
- Navbar icon tooltips and hover states
- Loading animation components (skeleton, spinner, progress)
- Scroll-reveal components trigger correctly

**3. Visual Regression Tests:**

- Screenshot animations at keyframes (start, middle, end)
- Compare reduced-motion vs full-motion states
- Test across 3 viewports: 320px, 768px, 1440px
- Dark mode + light mode visual consistency

**4. Performance Tests:**

- Lighthouse CI on every PR (Performance > 90)
- Core Web Vitals monitoring (LCP, INP, CLS thresholds)
- Animation frame rate testing (Chrome DevTools Performance)
- Bundle size monitoring (fail CI if animations add > 15KB)

**5. Accessibility Tests:**

- axe-core automated scans on all pages
- Manual keyboard navigation testing
- Screen reader testing (NVDA, VoiceOver)
- Reduced motion preference testing
- Focus indicator visibility testing

**6. E2E Tests (Playwright) - Critical User Journeys:**

The following user journeys MUST pass before Phase 6 exit. Each journey includes specific assertions, viewport requirements, and acceptance criteria.

---

**Journey 1: First-Time Visitor Discovery Flow**

**Persona:** New student discovering the textbook
**Viewport:** Desktop (1440px), Tablet (768px), Mobile (375px)
**Duration:** ~2 minutes per viewport

**Steps:**
1. Navigate to home page (`/`)
2. **Assert:** Hero section animates in (fade-up, staggered text)
3. **Assert:** LCP ≤ 2.5s (measure with Performance API)
4. Scroll down slowly (100px increments)
5. **Assert:** Section reveal animations trigger at 20% visibility
6. **Assert:** Feature cards have hover lift effect on desktop
7. **Assert:** No horizontal overflow at any scroll position
8. **Assert:** CLS ≤ 0.1 during scroll animations
9. Click "Get Started" CTA button
10. **Assert:** Button has press animation (scale down)
11. **Assert:** Page transition is smooth (no flash/jank)
12. **Assert:** Navigation completes within 500ms

**Playwright Test Skeleton:**
```typescript
test('first-time visitor discovery flow', async ({ page }) => {
  await page.goto('/');

  // Hero animation
  await expect(page.locator('[data-testid="hero-title"]')).toBeVisible();
  await expect(page.locator('[data-testid="hero-title"]')).toHaveCSS('opacity', '1');

  // Scroll and check section reveals
  for (let i = 0; i < 5; i++) {
    await page.mouse.wheel(0, 300);
    await page.waitForTimeout(100);
  }

  // Check CLS
  const cls = await page.evaluate(() => {
    return new Promise(resolve => {
      new PerformanceObserver(list => {
        const entries = list.getEntries();
        resolve(entries.reduce((sum, e) => sum + e.value, 0));
      }).observe({ type: 'layout-shift', buffered: true });
    });
  });
  expect(cls).toBeLessThan(0.1);

  // CTA interaction
  await page.click('[data-testid="cta-get-started"]');
  await expect(page).toHaveURL(/docs|getting-started/);
});
```

---

**Journey 2: Chatbot Panel Full Interaction Cycle**

**Persona:** Student seeking help with ROS concepts
**Viewport:** Desktop (1440px), Tablet (768px)
**Duration:** ~3 minutes per viewport

**Steps:**
1. Navigate to any chapter page (`/docs/module-1/introduction`)
2. **Assert:** Chatbot toggle button visible in bottom-right corner
3. **Assert:** Toggle button has accessible label (`aria-label="Open chat"`)
4. Click chatbot toggle button
5. **Assert:** Panel slides in from right with smooth animation (300ms)
6. **Assert:** Panel width is 400px (default) on desktop
7. **Assert:** Focus moves to chat input field automatically
8. **Assert:** No layout shift on main content (CLS = 0)
9. Type question: "What is a ROS publisher?"
10. **Assert:** Input field accepts text, has focus indicator
11. Press Enter or click Send button
12. **Assert:** User message appears with fade-in animation
13. **Assert:** Typing indicator (3-dot animation) appears
14. **Assert:** `aria-live` region announces "Generating response"
15. Wait for AI response (mock in test, max 5s timeout)
16. **Assert:** Response message animates in (slide-up + fade)
17. **Assert:** Typing indicator disappears
18. Scroll up in chat history (if multiple messages)
19. **Assert:** "Jump to latest" button appears when scrolled > 200px
20. Click "Jump to latest" button
21. **Assert:** Smooth scroll to bottom of chat
22. Click search icon in chat panel header
23. **Assert:** Search input field appears with focus
24. Type "publisher" in search
25. **Assert:** Matching messages are highlighted
26. Press Escape key
27. **Assert:** Search closes, returns to normal view
28. Click minimize button
29. **Assert:** Panel collapses to 64px height bar with icon
30. **Assert:** Unread badge appears if new messages arrive
31. Click minimized bar to expand
32. **Assert:** Panel expands with smooth animation
33. Drag resize handle on left edge
34. **Assert:** Panel resizes (min 320px, max 600px)
35. **Assert:** Resize is smooth (no jank)
36. Click close button (X)
37. **Assert:** Panel slides out with animation
38. **Assert:** Focus returns to toggle button
39. **Assert:** Toggle button now shows unread count badge if applicable

**Playwright Test Skeleton:**
```typescript
test('chatbot panel full interaction cycle', async ({ page }) => {
  await page.goto('/docs/module-1/introduction');

  // Open panel
  const toggleBtn = page.locator('[data-testid="chat-toggle"]');
  await expect(toggleBtn).toHaveAttribute('aria-label', 'Open chat');
  await toggleBtn.click();

  // Panel animation complete
  const panel = page.locator('[data-testid="chat-panel"]');
  await expect(panel).toBeVisible();
  await expect(panel).toHaveCSS('transform', 'none'); // animation complete

  // Focus management
  const input = page.locator('[data-testid="chat-input"]');
  await expect(input).toBeFocused();

  // Send message
  await input.fill('What is a ROS publisher?');
  await input.press('Enter');

  // User message visible
  await expect(page.locator('[data-testid="user-message"]').last()).toContainText('ROS publisher');

  // Typing indicator
  await expect(page.locator('[data-testid="typing-indicator"]')).toBeVisible();

  // Wait for response (mocked in test environment)
  await expect(page.locator('[data-testid="assistant-message"]').last()).toBeVisible({ timeout: 10000 });

  // Minimize
  await page.click('[data-testid="chat-minimize"]');
  await expect(panel).toHaveCSS('height', '64px');

  // Close
  await page.click('[data-testid="chat-close"]');
  await expect(panel).not.toBeVisible();
  await expect(toggleBtn).toBeFocused();
});
```

---

**Journey 3: Navbar Navigation with Animations**

**Persona:** Returning student navigating between modules
**Viewport:** Desktop (1440px), Tablet (768px), Mobile (375px)
**Duration:** ~2 minutes per viewport

**Steps:**
1. Navigate to home page (`/`)
2. **Assert:** Navbar is visible and fixed at top
3. **Assert:** All navbar icons have consistent size (24x24px)
4. Hover over navbar icon (desktop only)
5. **Assert:** Tooltip appears with accessible label
6. **Assert:** Icon has subtle scale/opacity transition
7. Click module dropdown/link
8. **Assert:** Dropdown animates open (fade + slide)
9. **Assert:** Dropdown items have staggered entrance
10. Navigate keyboard: Tab through navbar items
11. **Assert:** Focus indicators visible on each item
12. **Assert:** Focus order matches visual order
13. Press Enter on focused nav item
14. **Assert:** Page transition animates smoothly
15. **Assert:** New page content animates in
16. On mobile (375px): Click hamburger menu icon
17. **Assert:** Mobile drawer slides in from right
18. **Assert:** Overlay appears with fade animation
19. **Assert:** Focus trapped within drawer
20. Press Escape key
21. **Assert:** Drawer closes, focus returns to hamburger

**Playwright Test Skeleton:**
```typescript
test('navbar navigation with animations', async ({ page }) => {
  await page.goto('/');

  // Desktop tooltip
  await page.setViewportSize({ width: 1440, height: 900 });
  const navIcon = page.locator('[data-testid="nav-icon-docs"]');
  await navIcon.hover();
  await expect(page.locator('[role="tooltip"]')).toBeVisible();

  // Keyboard navigation
  await page.keyboard.press('Tab');
  await expect(page.locator(':focus')).toHaveCSS('outline-style', /solid|auto/);

  // Mobile hamburger
  await page.setViewportSize({ width: 375, height: 667 });
  await page.click('[data-testid="hamburger-menu"]');
  await expect(page.locator('[data-testid="mobile-drawer"]')).toBeVisible();

  // Escape closes drawer
  await page.keyboard.press('Escape');
  await expect(page.locator('[data-testid="mobile-drawer"]')).not.toBeVisible();
});
```

---

**Journey 4: Chapter Reading with Dark Mode and RTL**

**Persona:** Urdu-speaking student reading in dark mode
**Viewport:** Desktop (1440px), Tablet (768px)
**Duration:** ~2 minutes per viewport

**Steps:**
1. Navigate to any chapter (`/docs/module-2/ros-publishers`)
2. Toggle dark mode via navbar button
3. **Assert:** Theme transition is smooth (no flash)
4. **Assert:** All animations maintain correct colors in dark mode
5. **Assert:** Chatbot toggle button visible in dark mode
6. Toggle language to Urdu
7. **Assert:** Page direction changes to RTL
8. **Assert:** Animations work correctly in RTL (icons flip, slide directions reverse)
9. **Assert:** Chatbot panel docks to LEFT side in RTL mode
10. **Assert:** Navbar layout mirrors correctly
11. Open chatbot panel
12. **Assert:** Panel opens from LEFT in RTL mode
13. **Assert:** Text input is right-aligned
14. **Assert:** Message bubbles align correctly for RTL
15. Close panel, toggle back to English
16. **Assert:** Smooth transition back to LTR

**Playwright Test Skeleton:**
```typescript
test('chapter reading with dark mode and RTL', async ({ page }) => {
  await page.goto('/docs/module-2/ros-publishers');

  // Dark mode toggle
  await page.click('[data-testid="theme-toggle"]');
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');

  // Language switch to Urdu
  await page.click('[data-testid="language-selector"]');
  await page.click('[data-testid="lang-ur"]');

  // RTL verification
  await expect(page.locator('html')).toHaveAttribute('dir', 'rtl');

  // Chatbot in RTL
  await page.click('[data-testid="chat-toggle"]');
  const panel = page.locator('[data-testid="chat-panel"]');
  // In RTL, panel should be on left side
  const panelBox = await panel.boundingBox();
  expect(panelBox.x).toBeLessThan(100); // Near left edge
});
```

---

**Journey 5: Reduced Motion Accessibility**

**Persona:** User with motion sensitivity (vestibular disorder)
**Viewport:** Desktop (1440px)
**Duration:** ~2 minutes

**Steps:**
1. Enable `prefers-reduced-motion: reduce` in browser/OS settings
2. Navigate to home page (`/`)
3. **Assert:** Hero section appears instantly (no animation)
4. **Assert:** OR hero section uses opacity-only fade (no movement)
5. Scroll down
6. **Assert:** Section reveals are instant (no scroll animations)
7. **Assert:** OR sections use simple opacity transitions
8. Click CTA button
9. **Assert:** Button feedback is instant or opacity-only
10. **Assert:** Page transitions are instant or cross-fade only
11. Open chatbot panel
12. **Assert:** Panel appears instantly (no slide animation)
13. **Assert:** OR panel uses opacity fade only
14. Send a message
15. **Assert:** Messages appear instantly (no slide-up)
16. **Assert:** Typing indicator uses opacity pulse, not bouncing dots
17. **Assert:** All UI remains fully functional without motion
18. **Assert:** No user-initiated animation runs for > 5 seconds

**Playwright Test Skeleton:**
```typescript
test('reduced motion accessibility', async ({ page }) => {
  // Emulate reduced motion preference
  await page.emulateMedia({ reducedMotion: 'reduce' });

  await page.goto('/');

  // Hero should be visible immediately
  await expect(page.locator('[data-testid="hero-title"]')).toBeVisible();

  // Check no transform animations
  const heroStyle = await page.locator('[data-testid="hero-title"]').evaluate(el => {
    return window.getComputedStyle(el).transform;
  });
  expect(heroStyle).toBe('none');

  // Chatbot opens instantly
  await page.click('[data-testid="chat-toggle"]');
  const panel = page.locator('[data-testid="chat-panel"]');
  await expect(panel).toBeVisible();

  // Verify no transition duration
  const panelTransition = await panel.evaluate(el => {
    return window.getComputedStyle(el).transitionDuration;
  });
  expect(parseFloat(panelTransition)).toBeLessThan(0.1);
});
```

---

**Journey 6: Responsive Breakpoint Transitions**

**Persona:** User switching between devices or resizing browser
**Viewport:** Dynamic (1440px → 768px → 375px → 768px → 1440px)
**Duration:** ~2 minutes

**Steps:**
1. Navigate to chapter page at 1440px desktop
2. Open chatbot panel
3. **Assert:** Panel is docked, width 400px
4. Resize viewport to 768px (tablet)
5. **Assert:** Panel adapts smoothly (no jank)
6. **Assert:** Panel may reduce max-width or stay fixed
7. Resize viewport to 375px (mobile)
8. **Assert:** Panel becomes full-screen overlay
9. **Assert:** OR panel minimizes automatically
10. **Assert:** No horizontal overflow
11. **Assert:** Touch targets remain ≥ 44x44px
12. Close panel on mobile
13. Resize back to 1440px
14. **Assert:** Layout restores correctly
15. **Assert:** Chatbot toggle returns to corner position
16. Check navbar at each breakpoint
17. **Assert:** Desktop: Full navbar with icons
18. **Assert:** Tablet: May condense or show hamburger
19. **Assert:** Mobile: Hamburger menu only
20. **Assert:** No content cut off at any breakpoint

**Playwright Test Skeleton:**
```typescript
test('responsive breakpoint transitions', async ({ page }) => {
  await page.goto('/docs/module-1/introduction');

  const viewports = [
    { width: 1440, height: 900, name: 'desktop' },
    { width: 768, height: 1024, name: 'tablet' },
    { width: 375, height: 667, name: 'mobile' },
  ];

  for (const vp of viewports) {
    await page.setViewportSize({ width: vp.width, height: vp.height });

    // No horizontal overflow
    const scrollWidth = await page.evaluate(() => document.documentElement.scrollWidth);
    const clientWidth = await page.evaluate(() => document.documentElement.clientWidth);
    expect(scrollWidth).toBeLessThanOrEqual(clientWidth);

    // Touch targets on mobile
    if (vp.name === 'mobile') {
      const buttons = await page.locator('button, a, [role="button"]').all();
      for (const btn of buttons.slice(0, 10)) { // Check first 10
        const box = await btn.boundingBox();
        if (box) {
          expect(box.width).toBeGreaterThanOrEqual(44);
          expect(box.height).toBeGreaterThanOrEqual(44);
        }
      }
    }
  }
});
```

---

**Journey 7: Full Learning Session (End-to-End Happy Path)**

**Persona:** Graduate student completing a study session
**Viewport:** Desktop (1440px)
**Duration:** ~5 minutes

**Steps:**
1. Navigate to home page (`/`)
2. **Assert:** Hero animation completes
3. Click "Browse Modules" in navbar
4. **Assert:** Dropdown animates open
5. Select "Module 3: NVIDIA Isaac"
6. **Assert:** Page transitions smoothly to module page
7. Click first chapter in sidebar
8. **Assert:** Chapter content loads with skeleton/shimmer
9. **Assert:** Content fades in when ready
10. Scroll through chapter content
11. **Assert:** Images lazy load with fade-in
12. **Assert:** Code blocks have syntax highlighting
13. Open chatbot panel
14. **Assert:** Panel opens with slide animation
15. Ask: "Explain this code example"
16. **Assert:** Message sent, typing indicator shows
17. **Assert:** Response received and displayed
18. Click "Bookmark" button on chapter
19. **Assert:** Bookmark icon animates (heart fill, star pop, etc.)
20. Navigate to next chapter via "Next" button
21. **Assert:** Page transition with content animation
22. Toggle dark mode
23. **Assert:** Smooth theme transition, chatbot adapts
24. Close chatbot panel
25. **Assert:** Panel slides out
26. Navigate to bookmarks page (if exists)
27. **Assert:** Bookmarked chapter appears in list
28. Log out (if authenticated)
29. **Assert:** Logout confirmation with animation

**Playwright Test Skeleton:**
```typescript
test('full learning session happy path', async ({ page }) => {
  await page.goto('/');

  // Navigate to module
  await page.click('[data-testid="nav-modules"]');
  await page.click('[data-testid="module-3-link"]');
  await expect(page).toHaveURL(/module-3/);

  // Read chapter
  await page.click('[data-testid="chapter-1-link"]');
  await expect(page.locator('[data-testid="chapter-content"]')).toBeVisible();

  // Chatbot interaction
  await page.click('[data-testid="chat-toggle"]');
  await page.fill('[data-testid="chat-input"]', 'Explain this code example');
  await page.press('[data-testid="chat-input"]', 'Enter');
  await expect(page.locator('[data-testid="assistant-message"]')).toBeVisible({ timeout: 15000 });

  // Bookmark
  await page.click('[data-testid="bookmark-btn"]');
  await expect(page.locator('[data-testid="bookmark-btn"]')).toHaveAttribute('aria-pressed', 'true');

  // Dark mode
  await page.click('[data-testid="theme-toggle"]');
  await expect(page.locator('html')).toHaveAttribute('data-theme', 'dark');

  // Close chat
  await page.click('[data-testid="chat-close"]');
  await expect(page.locator('[data-testid="chat-panel"]')).not.toBeVisible();
});
```

---

**Journey 8: Error Recovery and Graceful Degradation**

**Persona:** User experiencing network issues or service errors
**Viewport:** Desktop (1440px)
**Duration:** ~2 minutes

**Steps:**
1. Navigate to chapter page
2. Open chatbot panel
3. Simulate network offline (`page.route` to block API)
4. Send a message
5. **Assert:** Loading indicator shows
6. **Assert:** After timeout, error message displays
7. **Assert:** Error message is user-friendly, not technical
8. **Assert:** Retry button is available
9. **Assert:** Error state has appropriate animation (subtle shake or fade)
10. Restore network connection
11. Click Retry
12. **Assert:** Message sends successfully
13. Simulate slow network (3G throttling)
14. Navigate to new chapter
15. **Assert:** Skeleton loading states appear
16. **Assert:** Content loads progressively
17. **Assert:** No blank screens during loading
18. Simulate chatbot API 500 error
19. **Assert:** Error toast/message appears
20. **Assert:** Fallback content or retry option shown
21. **Assert:** Rest of page remains functional

**Playwright Test Skeleton:**
```typescript
test('error recovery and graceful degradation', async ({ page }) => {
  await page.goto('/docs/module-1/introduction');
  await page.click('[data-testid="chat-toggle"]');

  // Block API requests to simulate offline
  await page.route('**/api/chat/**', route => route.abort());

  await page.fill('[data-testid="chat-input"]', 'Test message');
  await page.press('[data-testid="chat-input"]', 'Enter');

  // Error state
  await expect(page.locator('[data-testid="chat-error"]')).toBeVisible({ timeout: 10000 });
  await expect(page.locator('[data-testid="retry-btn"]')).toBeVisible();

  // Restore and retry
  await page.unroute('**/api/chat/**');
  await page.click('[data-testid="retry-btn"]');
  await expect(page.locator('[data-testid="assistant-message"]')).toBeVisible({ timeout: 15000 });
});
```

---

**Journey 9: Keyboard-Only Navigation (Accessibility)**

**Persona:** User relying on keyboard navigation (motor disability or preference)
**Viewport:** Desktop (1440px)
**Duration:** ~3 minutes

**Steps:**
1. Navigate to home page, use keyboard only (no mouse)
2. Press Tab repeatedly
3. **Assert:** Focus moves through all interactive elements
4. **Assert:** Focus indicators visible on each element (3:1 contrast)
5. **Assert:** Skip link available and functional ("Skip to content")
6. Navigate to chapter page
7. Tab to chatbot toggle button
8. Press Enter to open panel
9. **Assert:** Panel opens, focus moves to input
10. Type message, press Enter to send
11. **Assert:** Message sent successfully
12. Press Tab to navigate within chat panel
13. **Assert:** Tab order: input → send → search → minimize → close
14. Press Arrow keys in message list
15. **Assert:** Can navigate between messages
16. Press Escape
17. **Assert:** Panel closes, focus returns to toggle
18. Tab to navbar items
19. Press Enter/Space to activate dropdown
20. **Assert:** Dropdown opens, arrow keys navigate options
21. Press Escape to close dropdown
22. Tab to theme toggle, press Enter
23. **Assert:** Theme changes via keyboard
24. Tab to language selector, change language
25. **Assert:** Language changes via keyboard

**Playwright Test Skeleton:**
```typescript
test('keyboard-only navigation', async ({ page }) => {
  await page.goto('/');

  // Skip link
  await page.keyboard.press('Tab');
  const skipLink = page.locator('[data-testid="skip-link"]');
  await expect(skipLink).toBeFocused();

  // Navigate to chatbot
  for (let i = 0; i < 20; i++) {
    await page.keyboard.press('Tab');
    const focused = await page.locator(':focus').getAttribute('data-testid');
    if (focused === 'chat-toggle') break;
  }

  // Open with Enter
  await page.keyboard.press('Enter');
  await expect(page.locator('[data-testid="chat-panel"]')).toBeVisible();
  await expect(page.locator('[data-testid="chat-input"]')).toBeFocused();

  // Type and send
  await page.keyboard.type('Hello from keyboard');
  await page.keyboard.press('Enter');
  await expect(page.locator('[data-testid="user-message"]').last()).toContainText('Hello from keyboard');

  // Escape to close
  await page.keyboard.press('Escape');
  await expect(page.locator('[data-testid="chat-panel"]')).not.toBeVisible();
  await expect(page.locator('[data-testid="chat-toggle"]')).toBeFocused();
});
```

---

**Journey 10: Performance Under Load (Stress Test)**

**Persona:** Power user with many interactions
**Viewport:** Desktop (1440px)
**Duration:** ~3 minutes

**Steps:**
1. Navigate to home page
2. Rapidly scroll up and down (10 times)
3. **Assert:** Animations remain smooth (no dropped frames logged)
4. **Assert:** No memory leaks (heap size stable)
5. Open and close chatbot panel rapidly (5 times)
6. **Assert:** Panel opens/closes correctly each time
7. **Assert:** No zombie animations or stuck states
8. Send 10 messages in quick succession
9. **Assert:** All messages queued and sent correctly
10. **Assert:** UI remains responsive (INP < 200ms)
11. Toggle dark mode rapidly (5 times)
12. **Assert:** Theme switches correctly each time
13. Resize browser rapidly between breakpoints
14. **Assert:** Layout adapts without breaking
15. Open 3 browser tabs with same page
16. **Assert:** Each tab functions independently
17. **Assert:** LocalStorage state syncs correctly across tabs
18. **Measure:** Total JavaScript heap < 100MB after test
19. **Measure:** No console errors during test

**Playwright Test Skeleton:**
```typescript
test('performance under load', async ({ page }) => {
  await page.goto('/');

  // Rapid scroll
  for (let i = 0; i < 10; i++) {
    await page.mouse.wheel(0, 500);
    await page.waitForTimeout(50);
    await page.mouse.wheel(0, -500);
    await page.waitForTimeout(50);
  }

  // Rapid panel toggle
  for (let i = 0; i < 5; i++) {
    await page.click('[data-testid="chat-toggle"]');
    await page.waitForTimeout(100);
    await page.click('[data-testid="chat-close"]');
    await page.waitForTimeout(100);
  }

  // Check for console errors
  const errors = [];
  page.on('console', msg => {
    if (msg.type() === 'error') errors.push(msg.text());
  });

  expect(errors).toHaveLength(0);

  // Memory check
  const metrics = await page.evaluate(() => performance.memory?.usedJSHeapSize);
  if (metrics) {
    expect(metrics).toBeLessThan(100 * 1024 * 1024); // 100MB
  }
});
```

---

**E2E Test Configuration Requirements:**

```typescript
// playwright.config.ts additions for Phase 6
{
  projects: [
    { name: 'Desktop Chrome', use: { viewport: { width: 1440, height: 900 } } },
    { name: 'Tablet Safari', use: { viewport: { width: 768, height: 1024 } } },
    { name: 'Mobile Chrome', use: { viewport: { width: 375, height: 667 } } },
    { name: 'Desktop Firefox', use: { viewport: { width: 1440, height: 900 } } },
    { name: 'Reduced Motion', use: {
      viewport: { width: 1440, height: 900 },
      reducedMotion: 'reduce'
    }},
  ],
  expect: {
    timeout: 10000, // Animations may take time
  },
  use: {
    trace: 'retain-on-failure',
    video: 'retain-on-failure',
  },
}
```

**CI Integration:**
- All 10 journeys must pass on every PR
- Fail fast on first critical journey failure
- Generate HTML report with screenshots at each step
- Record video for failed tests
- Performance metrics logged to dashboard

---

**Free Tier Sustainability:**

- **No additional API costs:** Animations are client-side only
- **Bundle size impact:** Stay within Docusaurus defaults (no heavy libraries)
- **CDN bandwidth:** Animation assets are inline (no external requests)
- **Build time:** Animation compilation adds < 10s to build

---

**Success Criteria:**

- [ ] All micro-interactions implemented (buttons, inputs, form validation)
- [ ] Page transitions smooth and consistent across routes
- [ ] Chatbot panel fully functional with all specified features
- [ ] Navbar icons consistent, accessible, with tooltips
- [ ] Home page animations polished and performant
- [ ] Core Web Vitals pass: LCP ≤ 2.5s, INP ≤ 200ms, CLS ≤ 0.1
- [ ] WCAG 2.2 AA accessibility audit passes
- [ ] Reduced motion support verified (manual testing)
- [ ] Responsive UX verified on mobile, tablet, desktop (no overflow)
- [ ] Visual regression tests pass (no unintended layout changes)
- [ ] Performance tests pass (Lighthouse > 90, no frame drops)
- [ ] Bundle size increase ≤ 15KB gzipped
- [ ] All E2E tests pass
- [ ] PHR created documenting UI enhancement decisions

---

**Does NOT Include (Out of Scope for Phase 6):**

- No redesign of information architecture (sidebar structure, page hierarchy)
- No heavy animation libraries (GSAP, Anime.js) unless clearly justified and approved
- No functional changes:
  - No backend logic modifications
  - No database schema changes
  - No API contract modifications
  - No authentication flow changes
  - No RAG pipeline modifications
  - No translation service changes
- No new features (only UI/UX polish for existing features)
- No significant color palette or typography changes
- No rebranding or logo changes

---

**Follow-Up Items (Post Phase 6):**

- Monitor user feedback on animation preferences
- A/B test chatbot panel default width
- Add animation toggle in user settings (beyond reduced-motion)
- Implement advanced micro-interactions based on user patterns
- Create animation design system documentation

---

**Risks & Mitigation:**

**Risk 1: Performance Degradation from Animations**

- **Impact:** High (poor user experience, failed Core Web Vitals)
- **Likelihood:** Medium
- **Mitigation:**
  - Strict bundle size budget (15KB max)
  - Performance testing on every PR
  - Use only GPU-accelerated properties (transform, opacity)
  - Implement frame rate monitoring in development
- **Kill Switch:** Feature flag `ENABLE_ANIMATIONS=false` to disable all non-essential animations

**Risk 2: Accessibility Regressions**

- **Impact:** High (WCAG compliance failure, excluded users)
- **Likelihood:** Low (if testing is thorough)
- **Mitigation:**
  - Automated axe-core testing on every PR
  - Manual screen reader testing for chatbot
  - Reduced motion support mandatory for all animations
  - Focus management testing checklist
- **Kill Switch:** Animations respect reduced-motion by default

**Risk 3: Cross-Browser Animation Inconsistencies**

- **Impact:** Medium (inconsistent experience)
- **Likelihood:** Medium
- **Mitigation:**
  - Test on Chrome, Firefox, Safari, Edge
  - Use autoprefixer for CSS
  - Feature detection before using modern APIs
  - Graceful degradation to static states
- **Kill Switch:** CSS-only fallbacks for all animations

**Risk 4: Chatbot Panel Layout Conflicts**

- **Impact:** Medium (overlapping content, unusable UI)
- **Likelihood:** Low
- **Mitigation:**
  - Fixed positioning with proper z-index layering
  - Responsive width constraints
  - Mobile full-screen mode to avoid conflicts
  - Comprehensive E2E testing across viewports
- **Kill Switch:** Disable docked panel, fall back to modal overlay

---

**Constitution Alignment:**

- ✅ Quality Over Speed (Principle II): Comprehensive testing, performance budgets, accessibility audits
- ✅ Smallest Viable Change (Principle III): Only UI polish, no functional changes, no over-engineering
- ✅ Accessibility & Inclusivity (Principle VI): WCAG 2.2 AA compliance, reduced motion support, keyboard navigation
- ✅ Observability & Measurability (Principle V): Core Web Vitals tracking, performance monitoring, bundle size limits
- ✅ Free Tier Sustainability (Principle VII): No additional API costs, minimal bundle impact
- ✅ Security by Default (Principle IV): No new attack surfaces, client-side only changes

---

### Phase 6 Exit Criteria (UI Enhancement & Polish)

- [ ] All micro-interactions implemented (buttons, inputs, form states)
- [ ] Page transitions smooth across all routes
- [ ] Chatbot panel fully functional:
  - [ ] Docked right-side positioning
  - [ ] Collapsible/minimize with smooth transitions
  - [ ] Resizable width (320-600px)
  - [ ] Unread badge with count
  - [ ] Jump to latest button
  - [ ] Search in chat history
  - [ ] Loading animation during response generation
- [ ] Navbar icons consistent with accessible tooltips
- [ ] Home page animations polished (hero, sections, scroll-based)
- [ ] Core Web Vitals pass: LCP ≤ 2.5s, INP ≤ 200ms, CLS ≤ 0.1
- [ ] WCAG 2.2 AA accessibility audit passes
- [ ] Reduced motion support verified
- [ ] Responsive UX verified (mobile, tablet, desktop)
- [ ] Visual regression tests pass
- [ ] Lighthouse scores: Performance > 90, Accessibility > 90
- [ ] Bundle size increase ≤ 15KB gzipped
- [ ] All 10 E2E user journeys pass (Playwright):
  - [ ] Journey 1: First-Time Visitor Discovery Flow
  - [ ] Journey 2: Chatbot Panel Full Interaction Cycle
  - [ ] Journey 3: Navbar Navigation with Animations
  - [ ] Journey 4: Chapter Reading with Dark Mode and RTL
  - [ ] Journey 5: Reduced Motion Accessibility
  - [ ] Journey 6: Responsive Breakpoint Transitions
  - [ ] Journey 7: Full Learning Session (Happy Path)
  - [ ] Journey 8: Error Recovery and Graceful Degradation
  - [ ] Journey 9: Keyboard-Only Navigation (Accessibility)
  - [ ] Journey 10: Performance Under Load (Stress Test)
- [ ] Production deployment successful
- [ ] Documentation complete (animation guidelines, component docs)
- [ ] PHR created documenting UI enhancement implementation

---

## GOVERNANCE

### Decision Authority

**Orchestrator Agent:**

- Final authority on task prioritization
- Resolves inter-agent conflicts
- Approves phase transitions (quality gate checks)

**Architecture Changes:**

- Require ADR creation (via `/sp.adr`)
- User approval required for major changes
- Document in `history/adr/`

**Spec Changes:**

- Must pass constitution compliance check
- Require Orchestrator approval
- Update PHR with rationale

### Escalation Paths

**Agent Conflicts:**

1. Agents attempt resolution via shared context
2. If unresolved, escalate to Orchestrator
3. Orchestrator decides and documents in PHR

**Ambiguity in Requirements:**

1. Agent identifies ambiguity
2. Uses "Human as Tool" strategy (ask user 2-3 targeted questions)
3. User provides clarification
4. Update spec and document in PHR

**Budget/Scope Exceeded:**

1. Agent halts work
2. Reports to Orchestrator with details
3. Orchestrator escalates to user for decision
4. Document decision in ADR if architectural

**Technical Blocker:**

1. Agent investigates and documents issue
2. Proposes 2-3 solutions with tradeoffs
3. Escalates to user for decision
4. Document in ADR if significant

### Skill Invocation Rules

**Requirement:** Skills MUST be invoked through designated agents

**Enforcement:**

- `mdx-writer` → Only via ContentWriter agent
- `qdrant-vectorstore` → Only via RAGArchitect agent
- `better-auth-setup` → Only via AuthEngineer agent

**Rationale:** Ensures domain expertise and prevents skill misuse

### Priority Framework

**Order of Priorities:**

1. **Core Functionality** - Book rendering, RAG chatbot, authentication
2. **Quality & Security** - Testing, OWASP compliance, performance
3. **User Experience** - Responsive design, accessibility, dark mode
4. **Bonus Features** - Personalization, translation, advanced analytics

**Principle:** Deliver working core before adding enhancements

---

## NON-FUNCTIONAL REQUIREMENTS (NFRs)

### Reliability

**Uptime Target:** 99% (monthly basis)
**Error Budget:** 1% downtime = ~7 hours/month

**Degradation Strategy:**

- RAG failure → Static FAQ fallback
- Auth service down → Guest mode
- Translation unavailable → English fallback

### Observability

**Logging:**

- Structured JSON logs
- Correlation IDs for request tracing
- Log levels: DEBUG, INFO, WARN, ERROR, CRITICAL
- Retention: 30 days

**Metrics:**

- API latency (p50, p95, p99)
- Error rates (per endpoint)
- RAG relevance scores (daily aggregate)
- User engagement (sessions, queries)

**Tracing:**

- OpenTelemetry for distributed tracing
- Trace RAG pipeline: embed → search → generate

### Alerting

**Critical Alerts:**

- API error rate > 5% (5-minute window) → Notify immediately
- RAG response time p95 > 5s → Notify immediately
- Database connection pool exhausted → Notify immediately

**Warning Alerts:**

- Lighthouse score drops below 85 → Notify daily digest
- Translation cache hit rate < 70% → Notify weekly

### Cost Management

**Free Tier Monitoring:**

- Qdrant: Alert at 80% of 1GB storage
- Neon: Alert at 80 compute hours (100h/month limit)
- OpenAI: Track API spend, alert at $50/month

**Unit Economics:**

- Embedding cost: ~$0.0001 per query (OpenAI text-embedding-3-small)
- LLM generation: ~$0.002 per query (GPT-4o-mini)
- Target: < $0.01 per user per month (at 10 queries/user/month)

---

## OPERATIONAL READINESS

### Runbooks

**Common Tasks:**

1. **Deploy New Version**
   - Run CI/CD pipeline (`git push` to main)
   - Verify deployment health checks
   - Monitor error rates for 15 minutes

2. **Roll Back Deployment**
   - Revert to previous GitHub Pages deployment
   - Redeploy backend container (previous tag)
   - Verify rollback success

3. **Re-Embed Content (Model Upgrade)**
   - Export all chapters from Docusaurus
   - Run batch embedding script
   - Upload to Qdrant with new model version metadata
   - Run relevance tests to validate

4. **Handle OpenAI Rate Limit**
   - Enable rate limiting at application layer
   - Queue requests with backoff
   - Display user-friendly message: "High demand, please retry in 1 minute"

### Deployment Strategy

**CI/CD Pipeline:**

1. Push to main branch → Trigger GitHub Actions
2. Run linters (ESLint, Prettier, Black, Flake8)
3. Run unit tests (Jest, Pytest)
4. Run integration tests (API, RAG)
5. Build Docusaurus site
6. Deploy to GitHub Pages (frontend)
7. Build and push Docker image (backend)
8. Deploy to Railway/Render

**Rollback Procedure:**

- Frontend: Revert GitHub Pages deployment (via Actions)
- Backend: Redeploy previous Docker image tag

### Feature Flags

**Use Cases:**

- Gradual rollout of personalization (A/B test)
- Emergency disable of translation service
- Beta features for select users

**Implementation:** Environment variables or simple config file

### Backward Compatibility

**API Versioning:**

- Version in URL path: `/api/v1/query`
- Maintain v1 for 6 months after v2 release
- Document deprecation in OpenAPI spec

---

## RISK ANALYSIS AND MITIGATION

### Risk 1: OpenAI API Cost Overrun

**Impact:** High (budget exhaustion)
**Likelihood:** Medium
**Blast Radius:** All RAG functionality

**Mitigation:**

- Implement rate limiting per user (10 queries/hour free, 50 queries/hour authenticated)
- Cache frequent queries (Redis, 1-hour TTL)
- Monitor spend daily, alert at $50/month threshold
- Fallback: Disable chatbot, show static FAQ

**Kill Switch:** Environment variable `ENABLE_RAG=false`

---

### Risk 2: Qdrant Free Tier Storage Limit (1GB)

**Impact:** High (RAG failure if exceeded)
**Likelihood:** Medium (depends on content growth)
**Blast Radius:** Vector search fails

**Mitigation:**

- Monitor storage usage (alert at 80%)
- Compress embeddings (dimensionality reduction if needed)
- Prune old/unused chunks
- Upgrade to paid tier if necessary (last resort)

**Kill Switch:** Fallback to keyword search (basic)

---

### Risk 3: Translation Quality Issues

**Impact:** Medium (poor UX for Urdu users)
**Likelihood:** Medium
**Blast Radius:** Urdu translation feature

**Mitigation:**

- Manual review by native speaker (sample chapters)
- Technical term glossary (100% preservation enforced)
- User feedback mechanism (report translation errors)
- Iterative improvement based on feedback

**Kill Switch:** Disable Urdu toggle, show English only

---

## EVALUATION AND VALIDATION

### Definition of Done (Per Phase)

**Criteria:**

1. All acceptance criteria met (from spec)
2. Tests pass (unit, integration, E2E as applicable)
3. Code review completed (peer or agent review)
4. Documentation updated (README, API docs, ADRs)
5. PHR created documenting work
6. Quality gate checklist passed

### Output Validation

**Format Validation:**

- PHRs follow template structure (no missing fields)
- ADRs use standard format (Context, Decision, Consequences)
- Code passes linters (ESLint, Prettier, Black, Flake8)

**Requirements Validation:**

- Unit tests validate business logic
- Integration tests validate system interactions
- E2E tests validate user journeys

**Safety Validation:**

- OWASP Top 10 checklist passed
- Dependency scanning (no critical vulnerabilities)
- Input sanitization verified

---

## ARCHITECTURAL DECISION RECORDS (ADR)

### ADR Creation Criteria

**Three-Part Test (ALL must be true):**

1. **Impact:** Long-term consequences? Affects framework, data model, API, security, or platform?
2. **Alternatives:** Multiple viable options were considered?
3. **Scope:** Cross-cutting and influences system design?

**If ALL true → Suggest ADR:**

```
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

### ADR Examples for This Project

**Expected ADRs:**

1. "Use Docusaurus v3 over Next.js for static site generation"
2. "Use Qdrant Cloud instead of self-hosted vector database"
3. "Use OpenAI Agents SDK instead of LangChain for RAG"
4. "Use Better Auth instead of NextAuth for authentication"
5. "Deploy frontend to GitHub Pages instead of Vercel"

### ADR Workflow

1. Agent detects significant decision during planning/implementation
2. Agent suggests ADR creation (waits for user consent)
3. User approves: `/sp.adr <title>`
4. Agent creates ADR in `history/adr/NNNN-<slug>.md`
5. ADR includes: Context, Decision, Alternatives Considered, Consequences, Status

---

## HUMAN AS TOOL STRATEGY

### Invocation Triggers

**Agent MUST ask user when:**

1. **Ambiguous Requirements:** User intent unclear → Ask 2-3 targeted clarifying questions
2. **Unforeseen Dependencies:** Discovered dependencies not in spec → Surface and ask for prioritization
3. **Architectural Uncertainty:** Multiple valid approaches with significant tradeoffs → Present options
4. **Completion Checkpoint:** Major milestone reached → Summarize work, confirm next steps

### Question Guidelines

**Best Practices:**

- Ask 2-3 focused questions (not 10+)
- Provide context for why you're asking
- Offer 2-3 options with tradeoffs (when applicable)
- State your recommendation (if you have one)

**Example:**

```
I'm implementing the RAG chatbot and need to decide on chunk size:

Options:
1. 512 tokens - Faster search, less context per chunk
2. 1024 tokens - Slower search, more context per chunk
3. Adaptive chunking - Complex, but optimal for varied content

Recommendation: Start with 512 tokens (option 1) for speed, then optimize based on relevance metrics.

What's your preference?
```

---

## EXECUTION CONTRACT (Per Request)

**For Every Request, Agent Must:**

1. **Confirm Surface & Success Criteria** (1 sentence)
   - Example: "Surface: ContentWriter agent. Success: Chapter 3 authored with runnable code examples."

2. **List Constraints, Invariants, Non-Goals**
   - Constraints: Free tier limits, TypeScript strict mode
   - Invariants: No secrets in code, 80% test coverage
   - Non-Goals: Not building CMS, not supporting other languages yet

3. **Produce Artifact with Acceptance Checks**
   - Create the deliverable (code, doc, spec)
   - Include inline checkboxes or test assertions

4. **Add Follow-Ups and Risks** (max 3 bullets)
   - Follow-up: "Integrate chatbot UI with backend API"
   - Risk: "OpenAI rate limit if > 100 queries/day"

5. **Create PHR**
   - Route to appropriate subdirectory (constitution, feature, or general)
   - Fill all template fields (no placeholders)

6. **Suggest ADR (if applicable)**
   - Test decision against three-part test
   - Suggest ADR creation, wait for user consent

---

## MINIMUM ACCEPTANCE CRITERIA (Per Deliverable)

**Every Deliverable Must Have:**

- Clear, testable acceptance criteria
- Explicit error paths and constraints stated
- Smallest viable change (no unrelated edits)
- Code references to modified/inspected files (format: `path:line_start-line_end`)

**Example:**

```
Deliverable: RAG query endpoint

Acceptance Criteria:
- [ ] POST /api/v1/query accepts JSON body: {"query": "string"}
- [ ] Returns JSON: {"answer": "string", "sources": ["string"], "confidence": float}
- [ ] Response time p95 < 3s (load tested)
- [ ] Handles empty query (400 error)
- [ ] Handles OpenAI timeout (503 error with retry message)

Code References:
- `backend/api/routes.py:42-78` - Query endpoint implementation
- `backend/services/rag.py:15-60` - RAG pipeline logic

Constraints:
- Must use OpenAI Agents SDK (not custom LangChain)
- Must log query + response with correlation ID

Non-Goals:
- Not implementing streaming responses (future enhancement)
- Not supporting multi-turn conversations yet
```

---

## FINAL PRINCIPLES SUMMARY

### Measurability Over Aspiration

- ✅ "TypeScript strict mode, 80% test coverage"
- ❌ "High quality code"

### Reversibility Where Possible

- Use feature flags for major UI changes
- Keep old API versions during migration (6-month deprecation)
- Document rollback procedures in runbooks

### Smallest Viable Change

- Don't over-engineer Phase 1
- Build incrementally with quality gates between phases
- Avoid premature abstractions (YAGNI principle)

### Human as Tool

- List explicit cases for user escalation (ambiguity, architecture, blockers)
- Define when agents should pause and ask
- Provide context and options, not just "What should I do?"

### Constitution Compliance

- All agents must check constitution before starting work
- All specification changes require constitution compliance check
- Orchestrator audits compliance at phase gates

---

## REFERENCES

**Documentation Locations:**

- Constitution: `.specify/memory/constitution.md`
- Feature Specs: `specs/<feature>/spec.md`
- Architecture Plans: `specs/<feature>/plan.md`
- Task Lists: `specs/<feature>/tasks.md`
- Prompt History: `history/prompts/` (constitution, feature-name, or general)
- ADRs: `history/adr/`
- Templates: `.specify/templates/`

**Key Contacts:**

- Orchestrator Agent: Primary coordination
- User: Human as Tool for clarifications

---

**Version:** 1.1.0
**Ratified:** 2025-12-10
**Last Amended:** 2025-12-27
