# ADR-010: Rule-Based Skill Classification Algorithm

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-22
- **Feature:** 1-personalization-engine
- **Context:** Personalization engine needs to classify users into skill levels (beginner/intermediate/advanced) to provide adaptive content and recommendations. The classification must be deterministic, explainable, and work within free tier constraints while supporting 1000+ users.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use a rule-based weighted scoring algorithm for skill classification with the following components:
- Classification Factors: Quiz scores (40%), completed chapters (30%), code interactions (20%), time spent per chapter (10%)
- Thresholds: Beginner (0-40), Intermediate (41-70), Advanced (71-100)
- Implementation: Python service in `classification_service.py` with deterministic scoring
- Storage: Single record per user in `skill_classifications` table
- Reclassification: Automatic update when user completes new chapters or quizzes

## Consequences

### Positive

- Deterministic and explainable classification (transparent scoring factors)
- Minimal computational overhead (<500ms classification time)
- No ML model training/inference costs or complexity
- Works within Neon free tier constraints (simple database queries)
- Easy to debug and adjust scoring weights/thresholds
- Consistent behavior across all users (no probabilistic variation)

### Negative

- Less sophisticated than ML-based classification
- Manual tuning required for scoring weights and thresholds
- May not capture subtle learning patterns that ML could detect
- Requires periodic manual review of classification accuracy
- Limited to predefined factors (no automatic feature discovery)

## Alternatives Considered

Alternative A: Machine Learning Classification
- Use scikit-learn with features like quiz performance, time spent, code attempts
- Why rejected: Requires model training, feature engineering, and increased computational overhead; violates free tier constraints and YAGNI principle

Alternative B: Quiz-Based Classification Only
- Classify users based solely on initial assessment quiz results
- Why rejected: Too static, doesn't adapt to user progress over time; single quiz performance may not reflect actual skill level

Alternative C: Hard-Coded Rules Without Weighting
- Use simple heuristics like "completed 3+ chapters = intermediate"
- Why rejected: Too rigid, doesn't account for quiz performance or engagement metrics; lacks granularity for nuanced classification

## References

- Feature Spec: [specs/1-personalization-engine/spec.md](specs/1-personalization-engine/spec.md)
- Implementation Plan: [specs/1-personalization-engine/plan.md](specs/1-personalization-engine/plan.md)
- Related ADRs: ADR-005 (Section Level Personalization)
- Evaluator Evidence: Classification algorithm defined in research.md with validation criteria