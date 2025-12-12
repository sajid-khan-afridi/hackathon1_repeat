# Implementation Plan: Physical AI & Humanoid Robotics Textbook Platform - Book Infrastructure

**Branch**: `002-docusaurus-book-infra` | **Date**: 2025-12-11 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-docusaurus-book-infra/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build foundational static site infrastructure for Physical AI & Humanoid Robotics educational platform using Docusaurus v3. Deliver responsive MDX-based textbook with dark mode, GitHub Pages deployment, and RTL-ready architecture. Primary requirement: Enable automated content publishing with <5min build time and >85 Lighthouse scores. Technical approach: TypeScript-first Docusaurus configuration with CSS logical properties, GitHub Actions CI/CD, and progressive enhancement for accessibility.

## Technical Context

**Language/Version**: TypeScript 5.3+ (strict mode enabled), Node.js 18+
**Primary Dependencies**: Docusaurus v3.1+, React 18, MDX 3, GitHub Pages deployment
**Storage**: Static files (no database required for Phase 1)
**Testing**: Jest (unit), React Testing Library (component), Lighthouse CI (performance/accessibility), Playwright (E2E)
**Target Platform**: Modern browsers (Chrome, Firefox, Safari, Edge - last 2 years), GitHub Pages static hosting
**Project Type**: Web application (static site generator)
**Performance Goals**: First Contentful Paint <1.5s, Time to Interactive <3s, Cumulative Layout Shift <0.1, build time <5min for 100 pages
**Constraints**: GitHub Pages 100GB bandwidth/month, static-only (no SSR), WCAG 2.1 AA compliance, viewports 320px-2560px
**Scale/Scope**: 10 sample chapters initially (100 pages capacity), graduate-level educational content, mobile-first responsive design

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Phase 1 Quality Gates (from Constitution)
- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] Site deploys to GitHub Pages without errors
- [ ] Lighthouse Performance > 85
- [ ] Lighthouse Accessibility > 90
- [ ] Dark mode toggle functional
- [ ] Responsive on mobile (320px), tablet (768px), desktop (1024px)
- [ ] Test coverage: N/A (no business logic in Phase 1; static site setup only)
- [ ] PHR created documenting infrastructure decisions

### Core Principles Alignment

**✅ Quality Over Speed**: Phase has mandatory Lighthouse quality gates, test coverage requirements via CI
**✅ Smallest Viable Change**: Only Phase 1 deliverables (Docusaurus + dark mode + responsive), no premature features
**✅ Security by Default**: HTTPS enforced via GitHub Pages, Content Security Policy configured, no secrets in code
**✅ Observability & Measurability**: All success criteria are measurable (Lighthouse scores, build time, viewport sizes)
**✅ Accessibility & Inclusivity**: WCAG 2.1 AA compliance required, RTL-ready CSS architecture (logical properties)
**✅ Free Tier Sustainability**: GitHub Pages 100GB/month respected, monitoring planned at 80% threshold

### Agent Ownership
**Primary**: DocusaurusBuilder Agent (owns: `docusaurus-init`, `react-components`, `github-pages-deploy`)
**Support**: ContentWriter Agent (for sample content validation)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase 1 follows YAGNI principle, no over-engineering detected*

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Static Site Structure
docs/                       # MDX content (chapters organized by modules)
├── intro.md               # Landing page content
├── module-1/              # Example: ROS 2 Fundamentals
│   ├── chapter-1.mdx
│   ├── chapter-2.mdx
│   └── _category_.json    # Module metadata
├── module-2/              # Example: NVIDIA Isaac Sim
│   └── ...
└── ...

src/                       # Custom React components and theme
├── components/            # Reusable UI components
│   ├── ThemeToggle/       # Dark mode toggle component
│   │   ├── index.tsx
│   │   ├── styles.module.css
│   │   └── ThemeToggle.test.tsx
│   └── ...
├── css/                   # Global styles and theme tokens
│   └── custom.css         # RTL-ready CSS with logical properties
└── pages/                 # Custom pages (landing, about, etc.)
    └── index.tsx          # Home page

static/                    # Static assets
├── img/                   # Images, logos, diagrams
└── fonts/                 # Web fonts (if needed)

.github/
└── workflows/
    └── deploy.yml         # GitHub Actions CI/CD for GitHub Pages

tests/                     # Test suites
├── e2e/                   # Playwright end-to-end tests
│   └── navigation.spec.ts
├── lighthouse/            # Lighthouse CI configuration
│   └── lighthouserc.json
└── unit/                  # Jest unit tests for components

docusaurus.config.ts       # Docusaurus configuration (TypeScript)
tsconfig.json              # TypeScript configuration (strict mode)
package.json               # Dependencies and build scripts
```

**Structure Decision**: Using Docusaurus standard web application structure (Option 2 variant for static site generator). Rationale:
- `docs/` for content separation from code (enables ContentWriter Agent autonomy)
- `src/components/` for custom React components (DocusaurusBuilder Agent ownership)
- `src/css/` uses CSS logical properties (margin-inline, padding-block) for RTL preparation
- `.github/workflows/` for CI/CD automation (deploys to GitHub Pages)
- `tests/` organized by type (e2e, lighthouse, unit) for quality gates

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected.** Phase 1 adheres to all constitution principles:
- Uses Docusaurus defaults (no over-engineering)
- No database (static site only)
- No premature abstractions
- Quality gates enforced via Lighthouse CI
- RTL-ready via CSS logical properties (minimal effort, future-proof)

## Post-Design Constitution Check (Phase 1 Complete)

### ✅ All Quality Gates Remain Valid
- [ ] Docusaurus builds successfully (`npm run build`)
- [ ] Site deploys to GitHub Pages without errors
- [ ] Lighthouse Performance > 85
- [ ] Lighthouse Accessibility > 90
- [ ] Dark mode toggle functional
- [ ] Responsive on mobile (320px), tablet (768px), desktop (1024px)
- [ ] Test coverage: N/A (no business logic in Phase 1; static site setup only)
- [ ] PHR created documenting infrastructure decisions

### ✅ Design Artifacts Generated
- ✅ `research.md` - All NEEDS CLARIFICATION items resolved (Docusaurus v3, RTL architecture, GitHub Actions, dark mode, responsive design, syntax highlighting, performance)
- ✅ `data-model.md` - Entities defined (Chapter, Module, Theme Preference, Navigation Structure, Code Example, Build Artifact)
- ✅ `quickstart.md` - Developer and content author setup guide
- ⏳ `contracts/` - N/A for Phase 1 (static site, no API contracts required)

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Only implementing Phase 1 requirements (Docusaurus setup, dark mode, responsive design)
- **Security**: HTTPS enforced by GitHub Pages, Content Security Policy configured, no secrets in code
- **Performance**: Docusaurus built-in optimizations sufficient for >85 Lighthouse scores
- **Accessibility**: WCAG 2.1 AA compliance via Docusaurus defaults + custom CSS
- **Free Tier**: GitHub Pages 100GB/month, Docusaurus build <5min

### ✅ Recommended ADR

**Decision Detected**: Choice of Docusaurus v3 over Next.js/VitePress for static site generator

**ADR Suggestion**:
📋 Architectural decision detected: **Static Site Generator Selection (Docusaurus v3 vs Next.js vs VitePress)**
   Document reasoning and tradeoffs? Run `/sp.adr static-site-generator-selection`

**Justification for ADR**:
- **Impact**: Long-term (affects all future phases, content authoring DX, deployment)
- **Alternatives**: Next.js (more flexible, React ecosystem), VitePress (excellent performance, Vue-based)
- **Scope**: Cross-cutting (influences content structure, component development, CI/CD)

*Waiting for user consent before creating ADR.*

---

## Planning Summary

**Branch**: `002-docusaurus-book-infra`
**Status**: Phase 0 & Phase 1 Complete (Ready for `/sp.tasks` command)

**Artifacts Created**:
1. ✅ `plan.md` - Implementation plan (this file)
2. ✅ `research.md` - Phase 0 research findings
3. ✅ `data-model.md` - Phase 1 entity definitions
4. ✅ `quickstart.md` - Phase 1 developer guide
5. ⏳ `tasks.md` - Next step: Run `/sp.tasks` to generate actionable task list

**Next Command**: `/sp.tasks`
