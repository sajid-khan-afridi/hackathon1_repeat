### Phase 5: Translation (Owner: TranslationService)

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

**Integration with Other Phases:**
- **Phase 1 (Infrastructure):** RTL-compatible Docusaurus theme, font loading strategy
- **Phase 2 (Content):** Technical term glossary populated during authoring
- **Phase 4A (Authentication):** Language preference stored in user profile
- **Phase 6 (Deployment):** Translation pre-generation in CI/CD pipeline
