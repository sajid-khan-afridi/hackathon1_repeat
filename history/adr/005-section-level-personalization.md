# ADR-005: Section-Level Personalization via PersonalizedSection Component

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-12
- **Feature:** 002-mdx-textbook-chapters
- **Context:** Need to provide personalized learning experiences for different user profiles (experience level, ROS familiarity, hardware access) within the robotics textbook content while maintaining a single source of truth and avoiding content duplication.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement section-level personalization using a React component-based approach within MDX files.

- **Component**: `PersonalizedSection` React component with TypeScript
- **Props**: `level` (beginner|intermediate|advanced), `rosFamiliarity` (boolean), `hardwareAccess` (boolean)
- **Implementation**: MDX wrapper component that conditionally renders content based on user profile
- **Storage**: Personalization variants stored as structured data in chapter frontmatter
- **Integration**: Works with Docusaurus build system and maintains static generation capability

## Consequences

### Positive

- Single source of truth for content - no duplication across different chapter variants
- Clean separation of concerns between content authoring and personalization logic
- Enables future A/B testing and content optimization
- Maintains static site generation benefits (SEO, performance)
- Easy to extend with additional personalization dimensions
- Preserves content maintainability - updates apply to all variants automatically

### Negative

- Increased complexity in content authoring workflow
- Requires authors to understand component-based personalization
- Build-time complexity grows with number of personalization combinations
- Debugging content display issues requires understanding user profile state
- Component coupling to MDX transformation pipeline

## Alternatives Considered

**Alternative A: Separate Chapter Files per Variant**
- Approach: Create different MDX files for each personalization combination (e.g., chapter-1-beginner.mdx, chapter-1-expert.mdx)
- Benefits: Simple implementation, no runtime complexity
- Rejected: Massive content duplication, maintenance nightmare, inconsistent updates across variants

**Alternative B: Inline Conditional Text Blocks**
- Approach: Use custom MDX syntax like `{if beginner}...{endif}` within content
- Benefits: Familiar syntax for authors, similar to templating languages
- Rejected: Makes MDX files messy, hard to read/maintain, doesn't scale with multiple dimensions

**Alternative C: Server-Side Content Adaptation**
- Approach: Dynamically generate personalized content at request time
- Benefits: Maximum flexibility, real-time adaptation
- Rejected: Breaks static generation, adds server complexity, hurts SEO and performance

## References

- Feature Spec: specs/002-mdx-textbook-chapters/spec.md
- Implementation Plan: specs/002-mdx-textbook-chapters/plan.md
- Related ADRs: ADR-001 (Frontend Platform Stack)
- Data Model: specs/002-mdx-textbook-chapters/data-model.md