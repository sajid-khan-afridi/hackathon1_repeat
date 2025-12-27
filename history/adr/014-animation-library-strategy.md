# ADR-014: Animation Library Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-27
- **Feature:** 007-enhance-ui
- **Context:** Phase 6 requires implementing high-quality animations across the platform including chatbot panel transitions, home page scroll reveals, micro-interactions, and loading states. The animation approach must meet strict performance requirements (60fps, LCP ≤ 2.5s, CLS ≤ 0.1), bundle size constraints (≤15KB gzipped), and WCAG 2.2 AA accessibility including `prefers-reduced-motion` support.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ Affects all UI animations
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ CSS, Framer Motion, GSAP, Anime.js
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ Home page, chatbot, navbar, all components
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a **CSS-first animation strategy with Framer Motion as selective fallback** for the UI enhancement phase.

- **Primary**: CSS animations and transitions (native, zero bundle cost)
- **Fallback**: Framer Motion (tree-shakeable, ~12KB gzipped when selectively imported)
- **Constraint**: Total animation bundle ≤ 15KB gzipped
- **Property Restriction**: GPU-accelerated properties only (`transform`, `opacity`)
- **Accessibility**: Mandatory `prefers-reduced-motion` support via CSS media query + React hook

### Usage Guidelines

**Use CSS when:**
- Simple fade, slide, or scale transitions
- Hover/focus states
- Loading animations (keyframe-based)
- Page transitions
- Any animation that can be expressed declaratively

**Use Framer Motion when:**
- Complex orchestration with staggered children and dynamic timing
- Gesture-based interactions (drag-to-resize, swipe)
- Shared layout animations (layout morphing between states)
- Exit animations with AnimatePresence

## Consequences

### Positive

- **Zero bundle cost for majority of animations** - CSS is native to browsers
- **Excellent performance** - GPU-accelerated compositor-only animations
- **Reduced motion built-in** - Single CSS media query disables all movement
- **Framework-agnostic foundation** - CSS animations work regardless of React version
- **Graceful fallback** - Animations fail gracefully to static states
- **Measurable compliance** - Easy to verify bundle size and frame rate
- **TypeScript support** - Framer Motion has excellent type definitions

### Negative

- **Two animation paradigms** - Developers must know when to use CSS vs Framer Motion
- **CSS complexity for orchestration** - Staggered animations require nth-child hacks
- **No exit animations in pure CSS** - Requires AnimatePresence for unmount animations
- **Testing overhead** - Must test both CSS and Framer Motion code paths
- **Framer Motion learning curve** - Team needs familiarity for complex cases

## Alternatives Considered

**Alternative A: CSS-Only (No JavaScript Library)**
- Approach: All animations via CSS keyframes and transitions
- Benefits: Zero bundle size, maximum performance, simplest mental model
- Rejected: Cannot handle exit animations, staggered orchestration too complex, no gesture support for resize handle

**Alternative B: Framer Motion-Only**
- Approach: Use Framer Motion for all animations
- Bundle: ~40KB full, ~15KB tree-shaken
- Benefits: Consistent API, excellent DX, gesture support, exit animations
- Rejected: Unnecessary overhead for simple animations, approaches bundle budget limit with minimal CSS

**Alternative C: GSAP (GreenSock Animation Platform)**
- Approach: Use GSAP for complex animations, CSS for simple ones
- Bundle: ~60KB minified
- Benefits: Most powerful animation library, excellent performance, timeline control
- Rejected: Too heavy for UI polish use case, overkill for simple transitions, license considerations

**Alternative D: Anime.js**
- Approach: Lightweight JavaScript animation library
- Bundle: ~17KB
- Benefits: Simple API, good performance, SVG animation support
- Rejected: Not React-optimized, no hooks integration, manual cleanup required

**Alternative E: React Spring**
- Approach: Physics-based animation library for React
- Bundle: ~25KB
- Benefits: Natural-feeling animations, spring physics, React-optimized
- Rejected: Framer Motion has better DX, larger community, more examples for our use cases

## References

- Feature Spec: specs/007-enhance-ui/spec.md
- Implementation Plan: specs/007-enhance-ui/plan.md
- Research Evidence: specs/007-enhance-ui/research.md (Section 1: Animation Library Strategy)
- Related ADRs: ADR-001 (Frontend Platform Stack - establishes React/TypeScript foundation)
- Constitution: Phase 6 Quality Gates (animation bundle ≤15KB, 60fps, WCAG 2.2 AA)
