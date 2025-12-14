# ADR-008: Chat History Schema Design

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-14
- **Feature:** 003-rag-chatbot-core
- **Context:** Need to persist chat conversations for the RAG chatbot to enable session continuity, conversation history, and source traceability. Requirements include: support both anonymous and authenticated users, enable efficient retrieval of conversation threads, track source citations for transparency, monitor token usage for cost management, and comply with 30-day retention policy within Neon PostgreSQL 0.5GB free tier.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Long-term consequence for data model, query patterns, storage efficiency, and user experience
     2) Alternatives: YES - Multiple viable schema designs (normalized vs denormalized, embedded vs separated citations)
     3) Scope: YES - Cross-cutting concern affecting backend persistence, API responses, and frontend state management
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Adopt a normalized relational schema with separate tables for sessions, messages, and citations, using PostgreSQL UUID primary keys and foreign key constraints for referential integrity.

**Schema Components:**

- **ChatSession Table**: Stores conversation threads with UUID identifier, optional user_id for authenticated users, session_token for anonymous users, activity timestamps, and JSONB metadata for extensible filters/preferences
- **ChatMessage Table**: Stores individual messages with role ('user' or 'assistant'), content text (1-10,000 chars), JSONB token usage tracking, optional confidence score for assistant messages, and cascade delete on session removal
- **SourceCitation Table**: Stores source references in separate normalized table with foreign key to message_id, chapter identifiers, relevance scores (0.0-1.0), text excerpts (≤1,000 chars), and position ordering (1-based)
- **Indexing Strategy**: Indexed on session tokens, user IDs, activity timestamps, message sessions, and citation message/chapter relationships
- **Auto-Purge**: Database function to delete sessions with last_activity_at > 30 days (scheduled daily via cron)

## Consequences

### Positive

- **Query Efficiency**: Normalized structure enables efficient retrieval of conversation threads without large JSONB parsing
- **Source Traceability**: Separate citations table allows querying which chapters are most referenced across all conversations
- **Referential Integrity**: Foreign key constraints ensure data consistency (orphaned citations/messages automatically cleaned up)
- **Storage Efficiency**: Normalized structure avoids duplicating citation data when same source appears in multiple messages
- **Flexible Querying**: Can query citations by chapter, message, or relevance score independently
- **Extensibility**: JSONB metadata fields enable adding new properties without schema migrations
- **Type Safety**: PostgreSQL CHECK constraints enforce data validation (role enum, character limits, score ranges)
- **Cost Management**: 30-day auto-purge keeps storage within Neon 0.5GB free tier
- **Analytics Ready**: Normalized structure supports future analytics queries (popular topics, citation patterns)

### Negative

- **Join Complexity**: Retrieving full conversation with citations requires 3-table JOIN (session → messages → citations)
- **Write Amplification**: Single message with N citations requires N+1 database writes (1 message + N citations)
- **Migration Overhead**: Schema changes to normalized tables require careful migration planning
- **Query Count**: Frontend may need separate queries to fetch session → messages → citations (3 round trips)
- **Index Maintenance**: Multiple indexes add storage overhead and slow down write operations
- **Cascade Deletes**: Session deletion cascades to messages and citations (could be slow for large sessions)

## Alternatives Considered

**Alternative Schema A: Denormalized with Embedded Citations**
- Structure: Store citations as JSONB array within chat_messages.citations field
- Benefits: Single-table queries, atomic writes, simpler schema
- Rejected: Cannot efficiently query citations by chapter, harder to analyze citation patterns, JSONB parsing overhead, duplicated citation data across messages

**Alternative Schema B: Document Store (MongoDB/Firestore)**
- Structure: Nested document structure with embedded messages and citations
- Benefits: Flexible schema, natural fit for chat history, excellent read performance
- Rejected: Adds second database technology (complexity), Neon PostgreSQL already available, loses ACID guarantees, harder to implement relational analytics

**Alternative Schema C: Single Table with EAV Pattern**
- Structure: Entity-Attribute-Value pattern with generic key-value table
- Benefits: Ultimate schema flexibility, no migrations needed
- Rejected: Poor query performance, complex application logic, loses type safety, difficult to maintain

**Alternative Retention: Time-Series Partitioning**
- Structure: Partition chat_sessions by month for efficient purging
- Benefits: Faster purge operations, better query performance on recent data
- Rejected: Over-engineering for expected data volume, added complexity, Neon free tier may not support partitioning

## References

- Feature Spec: specs/003-rag-chatbot-core/spec.md (Key Entities section)
- Implementation Plan: specs/003-rag-chatbot-core/plan.md (lines 272-280)
- Data Model: specs/003-rag-chatbot-core/data-model.md (complete schema definition)
- Related ADRs: ADR-007 (RAG LLM Architecture Stack - response generation that populates this schema)
