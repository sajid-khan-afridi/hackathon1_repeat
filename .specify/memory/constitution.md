<!--
SYNC IMPACT REPORT
==================
Version Change: Initial → 1.0.0
Constitution Type: MAJOR - Comprehensive project constitution created

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

Templates Requiring Updates:
- ✅ plan-template.md - Constitution Check section aligns with new principles
- ✅ spec-template.md - Requirements align with new standards
- ✅ tasks-template.md - Task organization reflects new quality/testing principles
- ⚠️ No command files reference outdated constitution structure

Follow-up Items:
- None - All placeholders filled with concrete project values

Last Generated: 2025-12-10
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
- Phase 6: All E2E tests pass, Lighthouse > 90, production deployment successful

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

### VI. Accessibility & Inclusivity (WCAG 2.1 AA)

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

### Phase 6 Exit Criteria (Integration & Deployment)

- [ ] All critical user journeys pass E2E tests (Playwright)
- [ ] Lighthouse scores: Performance > 90, Accessibility > 90
- [ ] Production deployment successful
- [ ] Monitoring and alerting configured
- [ ] Documentation complete (README, API docs, ADRs)
- [ ] PHR created documenting final integration

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

**Version:** 1.0.0
**Ratified:** 2025-12-10
**Last Amended:** 2025-12-10
