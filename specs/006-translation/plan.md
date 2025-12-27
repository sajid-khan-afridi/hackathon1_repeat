# Implementation Plan: Phase 5 Translation

**Branch**: `006-translation` | **Date**: 2025-12-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-translation/spec.md`

## Summary

Deliver Urdu translation for all 10 textbook chapters with RTL layout support, technical term preservation, and a bilingual glossary. Translations are generated at build/deploy time and cached indefinitely (invalidated only on source content change). The implementation includes a language toggle component, hover tooltips for technical terms, and graceful fallback to English when translations are unavailable.

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11+ (build scripts/translation)
**Primary Dependencies**: Docusaurus v3, React 18+, `urdu-translator` skill, `react-components` skill
**Storage**: Static files only (pre-built translations + .meta.json cache files), localStorage (user language preference)
**Testing**: Jest + React Testing Library (components), Playwright (E2E/visual regression), pytest (translation scripts)
**Target Platform**: Web (GitHub Pages static hosting), Modern browsers with RTL support
**Project Type**: Web application (static frontend with build-time translation generation)
**Performance Goals**: Translation < 5s/chapter (build), cached load < 500ms, toggle response < 200ms
**Constraints**: CLS < 0.1, WCAG 2.1 AA compliance, free tier limits (translation API quotas only - no Neon usage for this phase)
**Scale/Scope**: 10 chapters, 50+ glossary terms, 2 languages (English, Urdu)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Phase 5 Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines 1211-1218 -->
- [ ] All 10 chapters translated to Urdu
- [ ] Translation time < 5s per chapter (measured)
- [ ] 100% technical term preservation (automated check)
- [ ] RTL layout renders correctly (visual regression tests)
- [ ] Native speaker review passes for 3 sample chapters
- [ ] PHR created documenting translation approach

### Core Principles Alignment

**✅ Quality Over Speed**: Visual regression testing on 3 viewports, automated term preservation validation, native speaker review for 3 sample chapters before release
**✅ Smallest Viable Change**: Only Urdu translation (not other languages), build-time generation (no runtime translation service), static file caching
**✅ Security by Default**: No API keys in client code (build-time only), input sanitization on glossary data, no user-generated translations
**✅ Observability & Measurability**: SC-001 to SC-010 provide specific metrics (< 5s, 100%, CLS < 0.1), translation timing measured in CI
**✅ Accessibility & Inclusivity**: WCAG 2.1 AA compliant toggle, aria-live regions for language changes, RTL screen reader support, 44x44px touch targets
**✅ Free Tier Sustainability**: Build-time translation (one-time API cost), file-based caching (no Neon usage), localStorage for preferences

### Agent Ownership
**Primary**: TranslationService Agent (owns: `urdu-translator`, `react-components`)
**Support**: DocusaurusBuilder Agent (RTL CSS integration), ContentWriter Agent (glossary content)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase follows YAGNI principle. Build-time translation is simpler than runtime translation service.*

## Project Structure

### Documentation (this feature)

```text
specs/006-translation/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (minimal - static content)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
docs/
├── i18n/
│   └── ur/                      # Urdu translations (generated at build)
│       ├── docusaurus-plugin-content-docs/
│       │   └── current/         # Translated MDX chapters
│       └── docusaurus-theme-classic/
│           └── navbar.json      # UI string translations

src/
├── components/
│   ├── LanguageToggle/          # Language switcher component
│   │   ├── index.tsx
│   │   ├── LanguageToggle.module.css
│   │   └── LanguageToggle.test.tsx
│   ├── GlossaryTooltip/         # Technical term hover tooltip
│   │   ├── index.tsx
│   │   └── GlossaryTooltip.test.tsx
│   └── GlossaryPage/            # Searchable glossary page
│       ├── index.tsx
│       └── GlossaryPage.test.tsx
├── css/
│   └── rtl.css                  # RTL-specific styles (CSS logical properties)
├── hooks/
│   └── useLanguagePreference.ts # localStorage + profile sync hook
├── context/
│   └── LanguageContext.tsx      # Language state provider
└── data/
    └── glossary.json            # Bilingual technical term glossary

# Build Scripts
scripts/
├── translate-chapters.ts        # Build-time translation orchestrator
├── validate-terms.ts            # Technical term preservation checker
└── generate-content-hash.ts     # Content hash for cache invalidation

# Tests
tests/
├── unit/
│   └── translation/
├── component/
│   └── LanguageToggle.test.tsx
├── integration/
│   └── language-switch.test.ts
└── visual/
    └── rtl-regression.spec.ts   # Playwright visual regression
```

**Structure Decision**: Web application with Docusaurus i18n integration. Translations stored as static files in `docs/i18n/ur/`. Build scripts generate translations during CI/CD.

## Complexity Tracking

> No violations identified. Build-time translation is the simplest viable approach.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

## Post-Design Constitution Check (Phase 5 Complete)

### ✅ All Quality Gates Remain Valid
<!-- Copy same checklist from Constitution Check above -->
- [ ] All 10 chapters translated to Urdu
- [ ] Translation time < 5s per chapter (measured)
- [ ] 100% technical term preservation (automated check)
- [ ] RTL layout renders correctly (visual regression tests)
- [ ] Native speaker review passes for 3 sample chapters
- [ ] PHR created documenting translation approach

### ✅ Design Artifacts Generated
- ✅ `research.md` - RTL best practices, Docusaurus i18n patterns, translation API integration
- ✅ `data-model.md` - GlossaryTerm, TranslationCache, LanguagePreference entities
- ✅ `quickstart.md` - Local development setup for translation testing
- ⏳ `contracts/` - Minimal (static content, no runtime API)

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Build-time translation avoids runtime translation service complexity
- **Security**: API keys used only at build time, not exposed to client
- **Performance**: Static file serving, aggressive caching, instant language toggle
- **Accessibility**: WCAG 2.1 AA compliant, RTL screen reader tested
- **Free Tier**: One-time translation cost at build, indefinite caching

### ✅ Recommended ADRs

**Decision Detected**: Build-Time Translation vs Runtime Translation Service
- **Impact**: Eliminates runtime API dependency, reduces costs, simplifies architecture
- **Alternatives**: On-demand translation, hybrid approach
- **Scope**: Affects deployment pipeline, caching strategy, user experience

📋 Architectural decision detected: **build-time-translation-strategy**
   Document reasoning and tradeoffs? Run `/sp.adr build-time-translation-strategy`

*Waiting for user consent before creating ADRs.*

---
