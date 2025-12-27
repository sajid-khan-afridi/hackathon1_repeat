# Implementation Plan: Enhancing the UI

**Branch**: `007-enhance-ui` | **Date**: 2025-12-27 | **Spec**: [specs/007-enhance-ui/spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-enhance-ui/spec.md`

<!--
IMPORTANT: Branch numbering follows phase-based numbering from the constitution:
- Phase 6 (Integration & Deployment): 007-*
-->

## Summary

Deliver a polished, delightful user experience with high-quality animations, an upgraded chatbot panel interface ("Zoom-style"), improved navbar navigation, and enhanced visual feedback. This is a UI-only enhancement phase with no backend changes, focusing on Core Web Vitals compliance, WCAG 2.2 AA accessibility, and responsive design.

## Technical Context

**Language/Version**: TypeScript 5.6 (strict mode) + React 19 + CSS Modules
**Primary Dependencies**: Docusaurus 3.9.2, React 19, clsx, Lucide React (to be added)
**Storage**: localStorage (panel state/width), sessionStorage (minimize state)
**Testing**: Jest + React Testing Library (unit/component), Playwright 1.57 (E2E)
**Target Platform**: Web (Desktop 1024px+, Tablet 768px-1023px, Mobile 320px-767px)
**Project Type**: Web application (Docusaurus static site with React components)
**Performance Goals**: 60fps animations, LCP ≤ 2.5s, INP ≤ 200ms, CLS ≤ 0.1
**Constraints**: Animation bundle ≤ 15KB gzipped, respect `prefers-reduced-motion`
**Scale/Scope**: ~15 components to enhance, 10 E2E user journeys, 6 user stories

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Phase 6 Quality Gates (from Constitution)
<!-- Reference: .specify/memory/constitution.md lines 2313-2348 -->
- [ ] All micro-interactions implemented (buttons, inputs, form states)
- [ ] Page transitions smooth across all routes
- [ ] Chatbot panel fully functional:
  - [ ] Docked right-side positioning (left in RTL)
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
- [ ] All 10 E2E user journeys pass (Playwright)
- [ ] Production deployment successful
- [ ] Documentation complete (animation guidelines, component docs)
- [ ] PHR created documenting UI enhancement implementation

### Core Principles Alignment

**✅ Quality Over Speed**: Comprehensive E2E testing (10 journeys), performance budgets, accessibility audits required before exit
**✅ Smallest Viable Change**: UI polish only - no backend changes, no new features, no database schema changes
**✅ Security by Default**: Client-side only changes, no new attack surfaces, no API modifications
**✅ Observability & Measurability**: Core Web Vitals tracking, Lighthouse CI, frame rate monitoring
**✅ Accessibility & Inclusivity**: WCAG 2.2 AA compliance, reduced motion support, RTL support, keyboard navigation
**✅ Free Tier Sustainability**: No additional API costs, animations are client-side only, minimal bundle impact

### Agent Ownership
**Primary**: DocusaurusBuilder Agent (owns: `react-components`, `docusaurus-init`)
**Support**: UI Enhancement Specialist (animation expertise)
**Coordinator**: Orchestrator Agent (quality gate approval)

### Complexity Violations
*None identified - Phase follows YAGNI principle. UI-only changes with no architectural modifications.*

## Project Structure

### Documentation (this feature)

```text
specs/007-enhance-ui/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - animation strategy, library comparison
├── data-model.md        # Phase 1 output - component state interfaces
├── quickstart.md        # Phase 1 output - development setup guide
├── contracts/           # N/A - no API changes in Phase 6
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── ChatPanel/                    # NEW: Docked chat panel component
│   │   ├── ChatPanel.tsx
│   │   ├── ChatPanel.module.css
│   │   ├── ChatPanelHeader.tsx
│   │   ├── ChatPanelSearch.tsx
│   │   ├── ResizeHandle.tsx
│   │   ├── UnreadBadge.tsx
│   │   ├── JumpToLatest.tsx
│   │   └── index.ts
│   ├── animations/                    # NEW: Animation utilities
│   │   ├── useReducedMotion.ts
│   │   ├── useScrollReveal.ts
│   │   ├── useAnimationState.ts
│   │   ├── FadeIn.tsx
│   │   ├── SlideIn.tsx
│   │   ├── CountUp.tsx
│   │   └── index.ts
│   ├── HomepageFeatures/              # ENHANCE: Add hover lift, scroll reveal
│   │   ├── index.tsx
│   │   └── styles.module.css
│   ├── FloatingChatPopup/             # REFACTOR: Extract to ChatPanel
│   │   └── EnhancedFloatingChatPopup.tsx
│   ├── Navbar/                        # ENHANCE: Icon system, tooltips
│   │   ├── NavbarIcons.tsx            # NEW
│   │   ├── NavbarTooltip.tsx          # NEW
│   │   └── MobileDrawer.tsx           # NEW
│   ├── SkeletonLoader/                # NEW: Loading states
│   │   ├── SkeletonLoader.tsx
│   │   ├── TypingIndicator.tsx
│   │   └── ProgressBar.tsx
│   └── ThemeToggle/                   # ENHANCE: Smooth transition
│       └── index.tsx
├── pages/
│   └── index.tsx                      # ENHANCE: Hero animation, section reveals
├── css/
│   ├── animations.css                 # NEW: Global animation keyframes
│   └── reduced-motion.css             # NEW: Reduced motion overrides
└── hooks/
    ├── useChatPanelState.ts           # NEW: Panel state management
    └── usePersistedState.ts           # NEW: localStorage/sessionStorage

tests/
├── e2e/
│   ├── journey-01-first-visit.spec.ts
│   ├── journey-02-chatbot-panel.spec.ts
│   ├── journey-03-navbar-navigation.spec.ts
│   ├── journey-04-dark-mode-rtl.spec.ts
│   ├── journey-05-reduced-motion.spec.ts
│   ├── journey-06-responsive.spec.ts
│   ├── journey-07-learning-session.spec.ts
│   ├── journey-08-error-recovery.spec.ts
│   ├── journey-09-keyboard-navigation.spec.ts
│   └── journey-10-performance.spec.ts
├── unit/
│   ├── useReducedMotion.test.ts
│   ├── useChatPanelState.test.ts
│   └── animation-utils.test.ts
└── component/
    ├── ChatPanel.test.tsx
    ├── SkeletonLoader.test.tsx
    └── NavbarIcons.test.tsx
```

**Structure Decision**: Web application with enhanced React components. No backend changes - all modifications are in `src/` directory only.

## Complexity Tracking

> **No violations - Phase follows YAGNI principle**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| *None* | *N/A* | *N/A* |

---

## Post-Design Constitution Check (Phase 6 Complete)

### ✅ All Quality Gates Remain Valid
<!-- Copy same checklist from Constitution Check above -->
- [ ] All micro-interactions implemented (buttons, inputs, form states)
- [ ] Page transitions smooth across all routes
- [ ] Chatbot panel fully functional (docked, resizable, minimize, search, unread badge)
- [ ] Navbar icons consistent with accessible tooltips
- [ ] Home page animations polished (hero, sections, scroll-based)
- [ ] Core Web Vitals pass: LCP ≤ 2.5s, INP ≤ 200ms, CLS ≤ 0.1
- [ ] WCAG 2.2 AA accessibility audit passes
- [ ] Reduced motion support verified
- [ ] Responsive UX verified (mobile, tablet, desktop)
- [ ] Lighthouse scores: Performance > 90, Accessibility > 90
- [ ] Bundle size increase ≤ 15KB gzipped
- [ ] All 10 E2E user journeys pass

### ✅ Design Artifacts Generated
- ✅ `research.md` - Animation library comparison, CSS vs Framer Motion decision
- ✅ `data-model.md` - ChatPanelState, AnimationPreference, component interfaces
- ✅ `quickstart.md` - Local development setup, testing commands, preview guide
- ⏳ `contracts/` - N/A for Phase 6 (UI-only, no API changes)

### ✅ No New Complexity Violations

**Analysis**:
- **YAGNI Compliance**: Only implementing features specified in spec. No premature abstractions.
- **Security**: No new attack surfaces. Client-side only. No secrets or API changes.
- **Performance**: Strict budget (15KB gzipped, 60fps). GPU-accelerated properties only.
- **Accessibility**: WCAG 2.2 AA required. Reduced motion support mandatory.
- **Free Tier**: No API costs. Animations are client-side CSS/JS only.

### ✅ Recommended ADRs

**Decision Detected**: Animation Library Strategy
- **Impact**: Affects bundle size, maintainability, performance of all animations
- **Alternatives**: CSS-only vs Framer Motion vs GSAP
- **Scope**: Cross-cutting - affects all animated components

📋 Architectural decision detected: **Animation Library Strategy (CSS-first with Framer Motion fallback)**
   Document reasoning and tradeoffs? Run `/sp.adr animation-library-strategy`

*Waiting for user consent before creating ADRs.*

---
