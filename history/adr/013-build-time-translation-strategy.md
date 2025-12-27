# ADR-013: Build-Time Translation Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-27
- **Feature:** 006-translation
- **Context:** Phase 5 Translation requires generating Urdu translations for 10 textbook chapters. The system must decide when translations are generated (build-time vs runtime), how they are stored and cached, where translation API keys are managed, and how updates propagate. The decision impacts deployment pipeline complexity, API cost management, user experience latency, and free tier sustainability.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? ✅ Yes - affects deployment, caching, security
     2) Alternatives: Multiple viable options considered with tradeoffs? ✅ Yes - 3 alternatives analyzed
     3) Scope: Cross-cutting concern (not an isolated detail)? ✅ Yes - affects CI/CD, frontend, caching, security
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Generate all Urdu translations at build/deploy time with the following integrated components:

- **Timing**: Invoke `urdu-translator` skill during CI/CD pipeline before Docusaurus build
- **Storage**: Store translations as static MDX files in `docs/i18n/ur/` (Docusaurus native i18n)
- **Caching**: Cache indefinitely; invalidate only when source content hash changes (MD5)
- **API Security**: Translation API keys used only in CI/CD environment, never in client bundle
- **Deployment**: Pre-translated content deployed to GitHub Pages as static files
- **Updates**: Content changes trigger rebuild; translations regenerated for modified chapters only

## Consequences

### Positive

- **Zero runtime API dependency**: Users get instant translations without API latency
- **Free tier sustainability**: One-time API cost per chapter (not per user request)
- **Security**: API keys never exposed to client-side code
- **Simplicity**: No runtime translation service to maintain, monitor, or scale
- **Performance**: Static file serving (< 500ms load) vs runtime translation (potentially 3-5s)
- **Reliability**: Translations available even if external API is down
- **Cacheable**: CDN/browser caching works naturally for static files
- **Cost predictable**: Translation cost = chapters × translation rate (known at deploy)

### Negative

- **Delayed updates**: Content changes require rebuild to update translations
- **Build time increase**: Translation adds ~50-100s to CI/CD pipeline (10 chapters × 5s)
- **No dynamic content**: Cannot translate user-generated content or chat responses
- **Full rebuild for fixes**: Translation corrections require complete deploy cycle
- **Storage overhead**: Duplicate content in English + Urdu (acceptable for 10 chapters)

## Alternatives Considered

**Alternative A: On-Demand Runtime Translation**
- Translate content when user requests Urdu version
- Pros: Always fresh translations, no build overhead, can translate any content
- Why rejected:
  - Adds 3-5s latency per page load on first access
  - Exposes API keys to server-side code (security risk if SSR)
  - Per-request API costs could exceed free tier limits quickly
  - Requires fallback handling for API failures
  - More complex error handling and retry logic

**Alternative B: Hybrid Approach (Pre-translate + On-demand Fallback)**
- Pre-translate core chapters at build time
- Translate new/updated content on-demand with caching
- Pros: Best of both worlds for core content
- Why rejected:
  - Complexity: Two translation codepaths to maintain
  - Still requires runtime API infrastructure
  - Cache invalidation complexity (when does on-demand cache expire?)
  - Inconsistent user experience (some pages instant, some slow)
  - Overkill for 10 static chapters that change infrequently

**Alternative C: External Translation Service (Google Translate / DeepL)**
- Use external translation API instead of custom urdu-translator skill
- Pros: Potentially higher quality translations, no custom skill maintenance
- Why rejected:
  - Less control over technical term preservation
  - May not integrate well with MDX/code block handling
  - External service dependency with potential cost surprises
  - urdu-translator skill already exists and is optimized for this use case

## References

- [Feature Spec](../../specs/006-translation/spec.md) - FR-015: Build-time translation requirement
- [Implementation Plan](../../specs/006-translation/plan.md) - Technical context and structure
- [Research](../../specs/006-translation/research.md) - urdu-translator skill integration patterns
- [Constitution](../../.specify/memory/constitution.md) - Phase 5 quality gates, free tier sustainability principle
