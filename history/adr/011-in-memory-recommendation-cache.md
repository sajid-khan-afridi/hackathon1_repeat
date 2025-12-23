# ADR-011: In-Memory Caching for Recommendations

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-22
- **Feature:** 1-personalization-engine
- **Context:** Recommendation API needs to serve personalized chapter suggestions within 1s while supporting 100 concurrent users. The recommendation algorithm involves multi-factor scoring (skill match, prerequisites, user goals) that can be expensive to compute repeatedly. Must work within free tier infrastructure constraints.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use in-memory TTLCache for recommendation caching with the following components:
- Cache Implementation: Python's `functools.TTLCache` with 1-hour TTL
- Cache Key: `{user_id}:{skill_level}:{goals_hash}`
- Cache Size: Maximum 1000 entries (1 per user) ~1MB memory usage
- Cache Strategy: Write-through (compute once, cache for future requests)
- Cache Invalidation: Automatic expiration + manual invalidation on skill level changes
- Storage: In-memory per FastAPI instance (no external cache service)

## Consequences

### Positive

- Significant performance improvement (~70% reduction in DB queries)
- Zero additional infrastructure costs or services
- Simple implementation with minimal dependencies
- FastAPI instance isolation (no cross-instance cache pollution)
- Automatic cache cleanup prevents memory leaks
- Works within free tier memory constraints

### Negative

- Cache not shared across multiple FastAPI instances
- Limited to 1-hour TTL (no immediate propagation of updates)
- Memory-bound (cache lost on service restart)
- No advanced cache features (e.g., cache warming, statistics)
- Potential for duplicate computation across instances in scaled deployment

## Alternatives Considered

Alternative A: Redis Cloud Cache
- Use Redis Cloud for distributed caching with advanced features
- Why rejected: Additional cost ($5+/month), complexity in setup and maintenance; violates free tier sustainability principle

Alternative B: Database Materialized Views
- Create materialized views for pre-computed recommendations in Postgres
- Why rejected: Increased storage costs, complex refresh logic, still requires DB queries; Neon free tier has limited storage

Alternative C: No Caching
- Compute recommendations on every request
- Why rejected: Cannot meet 1s response time requirement at scale; would lead to poor user experience and potential DB overload

## References

- Feature Spec: [specs/1-personalization-engine/spec.md](specs/1-personalization-engine/spec.md)
- Implementation Plan: [specs/1-personalization-engine/plan.md](specs/1-personalization-engine/plan.md)
- Related ADRs: ADR-009 (FastAPI Custom Authentication Stack)
- Evaluator Evidence: Performance requirements and cache strategy defined in research.md