# ADR-001: Frontend Platform Stack for Documentation Site

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** 001-docusaurus-init
- **Context:** Need to select a frontend platform for building a static documentation site that supports book-like navigation, dark/light themes, accessibility (WCAG 2.1 AA), high Lighthouse scores (>85), and efficient content authoring for a Physical AI & Humanoid Robotics textbook.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt Docusaurus v3 as the primary documentation platform with TypeScript for type safety and React 18 for component architecture.

- **Framework:** Docusaurus v3 with React 18
- **Language:** TypeScript 5.x (strict mode)
- **Styling:** CSS Modules + CSS custom properties for theming
- **Content:** Markdown-based with MDX for interactive components
- **Performance:** SWC compiler + code splitting + lazy loading

## Consequences

### Positive

- Excellent developer experience with integrated documentation tools
- Built-in search, versioning, and internationalization support
- Optimal performance out-of-the-box for static sites
- Strong TypeScript integration and type safety
- Native dark mode support with CSS custom properties
- SEO-friendly with automatic sitemap generation
- Large ecosystem of plugins and community support

### Negative

- Vendor lock-in to Docusaurus architecture and conventions
- Limited flexibility for complex application logic
- Learning curve for custom theming beyond CSS customizations
- Build-time rendering limits dynamic functionality
- Plugin ecosystem dependencies may introduce maintenance overhead

## Alternatives Considered

**Alternative A: Next.js with Custom Documentation Components**
- Framework: Next.js 14 (App Router)
- Styling: Tailwind CSS + CSS Modules
- Benefits: Full application flexibility, API routes, dynamic features
- Rejected: Over-engineering for static documentation, more complexity, slower development

**Alternative B: Gatsby with MDX Plugin**
- Framework: Gatsby with GraphQL layer
- Styling: Styled-components or Emotion
- Benefits: GraphQL data layer, plugin ecosystem, React ecosystem
- Rejected: Steeper learning curve, GraphQL complexity for simple docs, longer build times

**Alternative C: Hugo with Custom Theme**
- Framework: Hugo (Go-based static site generator)
- Styling: SCSS with custom theme development
- Benefits: Fastest build times, simple deployment
- Rejected: Limited React ecosystem, custom theme development effort, no TypeScript support

## References

- Feature Spec: specs/001-docusaurus-init/spec.md
- Implementation Plan: specs/001-docusaurus-init/plan.md
- Related ADRs: ADR-002 (GitHub Pages Deployment)
- Research Evidence: specs/001-docusaurus-init/research.md