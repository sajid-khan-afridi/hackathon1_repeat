# Feature Specification: Phase 5 Translation

**Feature Branch**: `006-translation`
**Created**: 2025-12-27
**Status**: Draft
**Input**: User description: "phase-5 Translation"

## Clarifications

### Session 2025-12-27

- Q: When should cached translations expire or be invalidated? → A: Never expire; invalidate only on source content change
- Q: When should translations be generated (build-time vs on-demand)? → A: Pre-translate all chapters at build/deploy time

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Browse Chapters in Urdu (Priority: P1)

An Urdu-speaking graduate student studying Physical AI and Humanoid Robotics wants to read textbook chapters in their native language for better comprehension while preserving technical accuracy.

**Why this priority**: This is the core value proposition of translation - enabling Urdu-speaking learners to access educational content in their preferred language. Without this, the translation feature provides no value.

**Independent Test**: Can be fully tested by navigating to any chapter, switching language to Urdu, and verifying the content displays correctly in RTL layout with technical terms preserved. Delivers the primary value of multilingual learning.

**Acceptance Scenarios**:

1. **Given** a user is on an English chapter page, **When** they click the language toggle and select Urdu, **Then** the chapter content displays in Urdu with RTL text direction
2. **Given** a chapter contains technical terms (ROS, Python, NVIDIA Isaac, Gazebo, URDF), **When** translated to Urdu, **Then** all technical terms remain in English with Urdu context/explanation
3. **Given** a chapter contains code blocks, **When** translated to Urdu, **Then** code blocks remain unchanged (no translation of code)
4. **Given** a user views Urdu content on mobile (320px), **When** scrolling through the chapter, **Then** RTL layout renders without horizontal scrolling or text overflow

---

### User Story 2 - Language Preference Persistence (Priority: P2)

A returning user who prefers Urdu wants their language preference remembered across sessions so they don't need to manually switch language every visit.

**Why this priority**: Improves UX by reducing friction for repeat visitors. Essential for user retention but not blocking core translation functionality.

**Independent Test**: Can be tested by selecting Urdu, closing browser, reopening site, and verifying Urdu is still selected. Delivers convenience value for returning users.

**Acceptance Scenarios**:

1. **Given** a user selects Urdu as their preferred language, **When** they close and reopen the browser, **Then** the site loads in Urdu by default (via localStorage)

> **Deferred to Future Enhancement:**
> - Cross-device sync for authenticated users (requires backend API)
> - Guest-to-account preference migration (requires auth integration)

---

### User Story 3 - Access Technical Glossary (Priority: P3)

A student encounters an unfamiliar technical term in Urdu content and wants to quickly understand its meaning and English equivalent.

**Why this priority**: Enhances learning experience and builds vocabulary, but users can still learn from context without it. Supports but doesn't block primary translation use case.

**Independent Test**: Can be tested by hovering over a technical term in Urdu content and verifying tooltip shows English term with definition. Alternatively, navigating to glossary page and searching for a term.

**Acceptance Scenarios**:

1. **Given** a user is reading Urdu content, **When** they hover over a technical term, **Then** a tooltip displays the English term with a brief definition
2. **Given** a user navigates to the Glossary page, **When** they search for "ROS", **Then** they see the English term, Urdu transliteration, and full definition
3. **Given** a user is on the Glossary page, **When** they filter by category (e.g., "Robotics Frameworks"), **Then** only relevant terms are displayed

---

### User Story 4 - Graceful Fallback on Translation Error (Priority: P4)

When translation service is unavailable, users should still be able to access content in English with a clear explanation rather than seeing a broken page.

**Why this priority**: Ensures reliability and prevents frustration, but represents an edge case rather than the primary user journey.

**Independent Test**: Can be tested by simulating translation service failure and verifying English content displays with appropriate error banner.

**Acceptance Scenarios**:

1. **Given** translation service is temporarily unavailable, **When** a user requests Urdu content, **Then** English version is displayed with a banner "Translation temporarily unavailable. Showing English version."
2. **Given** a translation partially fails (some sections translated, some not), **When** displayed to the user, **Then** translated sections show in Urdu and untranslated sections show in English with visual distinction
3. **Given** translation service becomes available again, **When** user clicks "Retry" button, **Then** the page refreshes with full Urdu translation

---

### Edge Cases

- What happens when a chapter has no cached translation and the user is offline?
  - Display English content with message: "Offline mode. Translation unavailable."
- How does the system handle bidirectional text (mixed English-Urdu in same sentence)?
  - Use BiDi text handling with appropriate Unicode markers to maintain reading order
- What happens when a new chapter is published but not yet translated?
  - Display English with banner: "Translation in progress. Check back soon."
- How does the system handle very long technical terms that don't fit well in Urdu context?
  - Keep technical terms in English; provide glossary tooltip for definition

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide Urdu translation for all 10 textbook chapters with 100% content coverage
- **FR-002**: System MUST preserve all technical terms (ROS, Python, NVIDIA Isaac, Gazebo, URDF, etc.) in English within translated content
- **FR-003**: System MUST preserve code blocks, code snippets, and variable names unchanged during translation
- **FR-004**: System MUST render Urdu content in RTL (Right-to-Left) layout with proper text direction
- **FR-005**: System MUST provide a language toggle component accessible from all pages
- **FR-006**: System MUST persist language preference in localStorage for all users
- **FR-007**: ~~System MUST sync language preference with user profile for authenticated users~~ *[DEFERRED: Cross-device sync requires backend API integration]*
- **FR-008**: System MUST provide a bilingual technical glossary (English-Urdu)
- **FR-009**: System MUST display hover tooltips showing English terms when users hover over technical terms in Urdu content
- **FR-010**: System MUST cache translated chapters indefinitely, invalidating cache only when source chapter content changes (detected via content hash comparison)
- **FR-011**: System MUST fall back to English content when translation service is unavailable
- **FR-012**: System MUST display translated MDX frontmatter (title, description) while preserving metadata structure
- **FR-013**: System MUST load appropriate fonts (Noto Nastaliq Urdu or equivalent) for Urdu content rendering
- **FR-014**: System MUST announce language changes to screen readers via aria-live regions
- **FR-015**: System MUST generate all Urdu translations at build/deploy time, not on-demand at runtime

### Key Entities

- **Translation**: Represents a translated version of a chapter (source language, target language, content, technical terms, timestamp, cache status)
- **Technical Term**: An entry in the bilingual glossary (English term, Urdu transliteration, definition, category, related terms)
- **Language Preference**: User's selected language stored in localStorage (language code, last updated)
- **Translation Cache**: File-based cache using `.meta.json` files alongside translated MDX (chapter ID, language, source content hash, created date); cache invalidated when source content hash changes

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All 10 textbook chapters have complete Urdu translations (100% coverage)
- **SC-002**: Translation generation completes in under 5 seconds per chapter (at build/deploy time)
- **SC-003**: Cached translations load in under 500 milliseconds
- **SC-004**: 100% of technical terms (as defined in glossary) are preserved in English within Urdu content
- **SC-005**: RTL layout passes visual regression tests on 3 viewports (320px, 768px, 1024px) with zero layout breaks
- **SC-006**: Language toggle interaction responds in under 200 milliseconds
- **SC-007**: Layout shift when switching languages measures CLS < 0.1
- **SC-008**: Native speaker review passes for 3 sample chapters with score >= 80% on quality rubric (grammar, accuracy, readability)
- **SC-009**: Language toggle component passes WCAG 2.1 AA accessibility audit
- **SC-010**: Glossary contains at least 50 technical terms with complete bilingual definitions

## Assumptions

- The `urdu-translator` skill is available and configured with appropriate API keys
- Technical terms list can be derived from existing chapter content and ROS/robotics domain knowledge
- Noto Nastaliq Urdu font is available via Google Fonts or can be self-hosted
- Users have modern browsers that support CSS logical properties and RTL layouts
- Translation API costs are within free tier limits when using caching strategy
- Native speaker review can be conducted within project timeline

## Out of Scope

- Translation to languages other than Urdu
- Audio or video content translation
- Real-time translation chat interface
- User-contributed translations or crowdsourcing
- Computer-Assisted Translation (CAT) tools
- Multiple Urdu dialect variants (using standard Modern Urdu)

## Dependencies

- Phase 2 (Content Creation) must be complete with 10 MDX chapters available
- `urdu-translator` skill must be operational
- `react-components` skill for UI components
- Existing Docusaurus theme must support RTL CSS extensions

> **Architecture Decision (2025-12-27):**
> - **TranslationCache**: File-based storage using `.meta.json` files alongside translated MDX (not Postgres). Simpler, free, version-controlled.
> - **Language Preference**: Client-only localStorage for all users. No backend sync required. Cross-device sync deferred to future enhancement.

## Risks

1. **Translation Quality Issues**: Machine translation may produce grammatically incorrect or contextually inappropriate Urdu content
   - Mitigation: Native speaker review, technical term glossary enforcement, user feedback mechanism
   - Kill switch: Disable Urdu toggle, serve English-only

2. **RTL Layout Breaks**: Complex layouts may not render correctly when mirrored for RTL
   - Mitigation: CSS logical properties, visual regression testing, cross-browser testing
   - Kill switch: Fall back to LTR layout with Urdu text

3. **Translation API Cost Overrun**: Frequent translation requests may exceed free tier limits
   - Mitigation: Aggressive caching, pre-translate during deployment, rate limiting
   - Kill switch: Disable on-demand translation, serve cached only
