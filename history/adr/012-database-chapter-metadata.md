# ADR-012: Database-Stored Chapter Metadata

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-22
- **Feature:** 1-personalization-engine
- **Context:** Personalization engine requires structured chapter metadata for recommendations and progress tracking. Metadata includes prerequisites, difficulty level, topics, code languages, estimated duration, and learning objectives. Must support efficient querying for the recommendation algorithm while maintaining content authoring workflow.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Store chapter metadata in a dedicated database table with the following components:
- Schema: `chapter_metadata` table with JSON columns for flexible attributes
- Core Fields: chapter_id (PK), title, difficulty_level, estimated_minutes, topics (JSON), code_languages (JSON), prerequisites (JSON), learning_objectives (JSON)
- Seeding: Initial data loaded via SQL migration with 10 sample chapters
- Querying: Indexed on difficulty_level, topics, code_languages for fast lookups
- Maintenance: Admin API endpoints for metadata CRUD operations
- Consistency: Metadata referenced by FK in other tables (progress, recommendations)

## Consequences

### Positive

- Queryable metadata for efficient recommendation algorithms
- Structured data enables complex filtering and joins
- Centralized metadata management with data integrity
- Easy to extend metadata schema without migration (JSON columns)
- Supports reporting and analytics on chapter attributes
- Maintains separation between content and metadata

### Negative

- Content authors must update metadata separately from MDX files
- Potential synchronization issues between content and metadata
- Additional database storage overhead (minimal for 10 chapters)
- Requires admin interface or direct DB access for metadata updates
- Learning curve for content maintainers

## Alternatives Considered

Alternative A: MDX Frontmatter Metadata
- Store all metadata as frontmatter in each MDX chapter file
- Why rejected: Difficult to query across all chapters for recommendations, would require file system operations and parsing, not performant at scale

Alternative B: Hardcoded Metadata in Python
- Define chapter metadata as Python dictionaries/lists in the code
- Why rejected: Requires code deployments for metadata updates, violates separation of content and logic, not maintainable for non-technical authors

Alternative C: External CMS for Metadata
- Use a headless CMS or separate metadata service
- Why rejected: Adds infrastructure complexity and cost, overkill for 10 chapters, violates free tier constraints

## References

- Feature Spec: [specs/1-personalization-engine/spec.md](specs/1-personalization-engine/spec.md)
- Implementation Plan: [specs/1-personalization-engine/plan.md](specs/1-personalization-engine/plan.md)
- Data Model: [specs/1-personalization-engine/data-model.md](specs/1-personalization-engine/data-model.md)
- Related ADRs: ADR-004 (Static Site Generator Selection)
- Evaluator Evidence: Schema design and seeding strategy defined in data-model.md