# Data Model: RAG Chatbot Core

**Feature**: 003-rag-chatbot-core
**Date**: 2025-12-14
**Status**: Complete

## Overview

This document defines the data entities, relationships, and validation rules for the RAG Chatbot Core feature. The model is derived from the Key Entities section of the feature specification.

---

## Entity Relationship Diagram

```
┌─────────────────┐       ┌─────────────────┐       ┌─────────────────┐
│   ChatSession   │       │   ChatMessage   │       │ SourceCitation  │
├─────────────────┤       ├─────────────────┤       ├─────────────────┤
│ id (PK)         │──1:N──│ id (PK)         │──1:N──│ id (PK)         │
│ user_id (FK?)   │       │ session_id (FK) │       │ message_id (FK) │
│ session_token   │       │ role            │       │ chapter_id      │
│ created_at      │       │ content         │       │ chapter_title   │
│ last_activity_at│       │ tokens_used     │       │ relevance_score │
│ metadata        │       │ confidence      │       │ excerpt         │
└─────────────────┘       │ created_at      │       │ position        │
                          └─────────────────┘       └─────────────────┘

┌─────────────────┐       ┌─────────────────┐
│  RateLimitState │       │   QueryRequest  │ (API Model - not persisted)
├─────────────────┤       ├─────────────────┤
│ identifier (PK) │       │ query           │
│ query_count     │       │ user_id?        │
│ window_start    │       │ session_id?     │
│ limit_tier      │       │ filters?        │
└─────────────────┘       │ top_k?          │
                          └─────────────────┘

┌─────────────────┐
│  QueryResponse  │ (API Model - not persisted)
├─────────────────┤
│ answer          │
│ sources[]       │
│ confidence      │
│ session_id      │
│ tokens_used     │
└─────────────────┘
```

---

## Entities

### 1. ChatSession

Represents a conversation thread. Sessions can be anonymous (session-based) or tied to an authenticated user.

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, auto-generated | Unique session identifier |
| `user_id` | UUID | FK to users, nullable | User ID for authenticated sessions |
| `session_token` | VARCHAR(64) | NOT NULL, unique | Token for anonymous session identification |
| `created_at` | TIMESTAMPTZ | NOT NULL, default NOW() | Session creation timestamp |
| `last_activity_at` | TIMESTAMPTZ | NOT NULL, default NOW() | Last message timestamp |
| `metadata` | JSONB | default '{}' | Extensible metadata (filters, preferences) |

#### Validation Rules

- `session_token` must be cryptographically random (use `secrets.token_urlsafe(48)`)
- `last_activity_at` must be updated on every message
- Sessions older than 30 days are eligible for purge
- Either `user_id` OR `session_token` must be present for identification

#### State Transitions

```
Created → Active → Expired → Purged
   │         │
   │         └── last_activity_at > 30 days
   │
   └── On first message
```

#### SQL Schema

```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    session_token VARCHAR(64) NOT NULL UNIQUE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_activity_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}',

    CONSTRAINT valid_identification CHECK (
        user_id IS NOT NULL OR session_token IS NOT NULL
    )
);

CREATE INDEX idx_chat_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX idx_chat_sessions_token ON chat_sessions(session_token);
CREATE INDEX idx_chat_sessions_activity ON chat_sessions(last_activity_at);
```

---

### 2. ChatMessage

Individual message in a conversation. Stores both user queries and assistant responses.

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, auto-generated | Unique message identifier |
| `session_id` | UUID | FK to chat_sessions, NOT NULL | Parent session reference |
| `role` | VARCHAR(20) | NOT NULL, enum | Message author: 'user' or 'assistant' |
| `content` | TEXT | NOT NULL | Message text content |
| `tokens_used` | JSONB | default '{}' | Token usage: {input: N, output: M} |
| `confidence` | FLOAT | 0.0-1.0, nullable | Confidence score (assistant only) |
| `created_at` | TIMESTAMPTZ | NOT NULL, default NOW() | Message timestamp |

#### Validation Rules

- `role` must be one of: 'user', 'assistant'
- `content` must not be empty (min 1 character)
- `content` must not exceed 10,000 characters
- `confidence` only applies to assistant messages
- `tokens_used` structure: `{"input": int, "output": int, "total": int}`

#### SQL Schema

```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (LENGTH(content) > 0 AND LENGTH(content) <= 10000),
    tokens_used JSONB DEFAULT '{}',
    confidence FLOAT CHECK (confidence IS NULL OR (confidence >= 0.0 AND confidence <= 1.0)),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT confidence_only_for_assistant CHECK (
        role != 'user' OR confidence IS NULL
    )
);

CREATE INDEX idx_chat_messages_session ON chat_messages(session_id);
CREATE INDEX idx_chat_messages_created ON chat_messages(created_at);
```

---

### 3. SourceCitation

Reference to textbook content used in generating a response.

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, auto-generated | Unique citation identifier |
| `message_id` | UUID | FK to chat_messages, NOT NULL | Parent message reference |
| `chapter_id` | VARCHAR(100) | NOT NULL | Chapter identifier (e.g., 'module-1/chapter-2') |
| `chapter_title` | VARCHAR(255) | NOT NULL | Human-readable chapter title |
| `relevance_score` | FLOAT | 0.0-1.0, NOT NULL | Vector similarity score |
| `excerpt` | TEXT | NOT NULL | Relevant text excerpt from chapter |
| `position` | INT | NOT NULL | Order in citation list (1-based) |

#### Validation Rules

- `chapter_id` must match pattern: `module-N/chapter-M` or `module-N-topic/chapter-M-subtopic`
- `relevance_score` must be between 0.0 and 1.0
- `excerpt` must not exceed 1,000 characters
- `position` must be positive integer (1, 2, 3, ...)
- Citations are ordered by position (ascending)

#### SQL Schema

```sql
CREATE TABLE source_citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES chat_messages(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    chapter_title VARCHAR(255) NOT NULL,
    relevance_score FLOAT NOT NULL CHECK (relevance_score >= 0.0 AND relevance_score <= 1.0),
    excerpt TEXT NOT NULL CHECK (LENGTH(excerpt) <= 1000),
    position INT NOT NULL CHECK (position > 0),

    UNIQUE (message_id, position)
);

CREATE INDEX idx_source_citations_message ON source_citations(message_id);
CREATE INDEX idx_source_citations_chapter ON source_citations(chapter_id);
```

---

### 4. RateLimitState

Tracks user's API usage for rate limiting. Can be in-memory (Redis) or database.

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `identifier` | VARCHAR(100) | PK | User/session identifier |
| `query_count` | INT | NOT NULL, default 0 | Queries in current window |
| `window_start` | TIMESTAMPTZ | NOT NULL | Start of current rate limit window |
| `limit_tier` | VARCHAR(20) | NOT NULL | 'anonymous' or 'authenticated' |

#### Validation Rules

- `identifier` format: `user:{uuid}` for authenticated, `anon:{ip}` for anonymous
- `query_count` must be non-negative
- `limit_tier` must be one of: 'anonymous', 'authenticated'
- Window duration: 1 hour

#### Rate Limits (from spec)

| Tier | Limit | Window |
|------|-------|--------|
| anonymous | 10 queries | 1 hour |
| authenticated | 50 queries | 1 hour |

#### Implementation Note

For production, use Redis for rate limiting:
```python
# Redis key pattern
rate_limit:{identifier}:count
rate_limit:{identifier}:window_start
```

---

### 5. QueryRequest (API Model)

Incoming question from user. Not persisted; used for API request validation.

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `query` | string | 1-500 chars, required | User's question text |
| `user_id` | UUID | optional | Authenticated user identifier |
| `session_id` | UUID | optional | Existing session to continue |
| `filters` | object | optional | Search filters |
| `filters.module` | int | 1-10 | Module number filter |
| `filters.difficulty` | string | enum | 'beginner', 'intermediate', 'advanced' |
| `filters.tags` | string[] | optional | Topic tags |
| `top_k` | int | 1-10, default 5 | Number of sources to retrieve |

#### Validation Rules

- `query` must be 1-500 characters (FR-001)
- `query` must be sanitized for XSS/injection (FR-025)
- `top_k` must be between 1 and 10 (FR-003)
- `filters.module` must be valid module number if provided
- `filters.difficulty` must be one of allowed values if provided

#### Pydantic Model

```python
from pydantic import BaseModel, Field, field_validator
from typing import Optional, List
import uuid
import re

class QueryFilters(BaseModel):
    module: Optional[int] = Field(None, ge=1, le=10)
    difficulty: Optional[str] = Field(None, pattern='^(beginner|intermediate|advanced)$')
    tags: Optional[List[str]] = Field(default_factory=list)

class QueryRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=500)
    user_id: Optional[uuid.UUID] = None
    session_id: Optional[uuid.UUID] = None
    filters: Optional[QueryFilters] = None
    top_k: int = Field(5, ge=1, le=10)

    @field_validator('query')
    @classmethod
    def sanitize_query(cls, v: str) -> str:
        # Remove potential XSS/injection patterns
        v = re.sub(r'<[^>]*>', '', v)  # Strip HTML tags
        v = v.strip()
        return v
```

---

### 6. QueryResponse (API Model)

System's answer to a question. Not persisted directly; parts stored in ChatMessage and SourceCitation.

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `answer` | string | required | Generated answer text |
| `sources` | SourceCitation[] | required | List of source citations |
| `confidence` | float | 0.0-1.0, required | Answer confidence score |
| `session_id` | UUID | required | Chat session identifier |
| `tokens_used` | object | required | Token usage breakdown |
| `tokens_used.input` | int | required | Input tokens consumed |
| `tokens_used.output` | int | required | Output tokens generated |
| `tokens_used.total` | int | required | Total tokens |
| `filter_message` | string | optional | Message about filter behavior |

#### Pydantic Model

```python
from pydantic import BaseModel, Field
from typing import List, Optional
import uuid

class SourceCitationResponse(BaseModel):
    chapter_id: str
    chapter_title: str
    relevance_score: float = Field(..., ge=0.0, le=1.0)
    excerpt: str

class TokenUsage(BaseModel):
    input: int = Field(..., ge=0)
    output: int = Field(..., ge=0)
    total: int = Field(..., ge=0)

class QueryResponse(BaseModel):
    answer: str
    sources: List[SourceCitationResponse]
    confidence: float = Field(..., ge=0.0, le=1.0)
    session_id: uuid.UUID
    tokens_used: TokenUsage
    filter_message: Optional[str] = None
```

---

## Database Migrations

### Migration 001: Create Chat Tables

```sql
-- Migration: 001_create_chat_tables.sql
-- Description: Create tables for RAG chatbot chat history

BEGIN;

-- Enable UUID extension if not exists
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Create chat_sessions table
CREATE TABLE IF NOT EXISTS chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    session_token VARCHAR(64) NOT NULL UNIQUE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_activity_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'
);

CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_token ON chat_sessions(session_token);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_activity ON chat_sessions(last_activity_at);

-- Create chat_messages table
CREATE TABLE IF NOT EXISTS chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (LENGTH(content) > 0 AND LENGTH(content) <= 10000),
    tokens_used JSONB DEFAULT '{}',
    confidence FLOAT CHECK (confidence IS NULL OR (confidence >= 0.0 AND confidence <= 1.0)),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_chat_messages_session ON chat_messages(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_messages_created ON chat_messages(created_at);

-- Create source_citations table
CREATE TABLE IF NOT EXISTS source_citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES chat_messages(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    chapter_title VARCHAR(255) NOT NULL,
    relevance_score FLOAT NOT NULL CHECK (relevance_score >= 0.0 AND relevance_score <= 1.0),
    excerpt TEXT NOT NULL CHECK (LENGTH(excerpt) <= 1000),
    position INT NOT NULL CHECK (position > 0),
    UNIQUE (message_id, position)
);

CREATE INDEX IF NOT EXISTS idx_source_citations_message ON source_citations(message_id);
CREATE INDEX IF NOT EXISTS idx_source_citations_chapter ON source_citations(chapter_id);

COMMIT;
```

### Migration 002: Add Auto-Purge Function

```sql
-- Migration: 002_add_auto_purge.sql
-- Description: Add function and trigger for auto-purging old sessions

BEGIN;

-- Function to purge old sessions (called by cron/scheduled job)
CREATE OR REPLACE FUNCTION purge_old_chat_sessions()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
BEGIN
    WITH deleted AS (
        DELETE FROM chat_sessions
        WHERE last_activity_at < NOW() - INTERVAL '30 days'
        RETURNING id
    )
    SELECT COUNT(*) INTO deleted_count FROM deleted;

    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Note: Schedule this function to run daily using pg_cron or external scheduler
-- Example with pg_cron:
-- SELECT cron.schedule('purge-old-sessions', '0 3 * * *', 'SELECT purge_old_chat_sessions();');

COMMIT;
```

---

## TypeScript Types (Frontend)

```typescript
// types/chat.ts

export interface ChatSession {
  id: string;
  userId?: string;
  createdAt: string;
  lastActivityAt: string;
}

export interface ChatMessage {
  id: string;
  sessionId: string;
  role: 'user' | 'assistant';
  content: string;
  tokensUsed?: TokenUsage;
  confidence?: number;
  sources?: SourceCitation[];
  createdAt: string;
}

export interface SourceCitation {
  id?: string;
  chapterId: string;
  chapterTitle: string;
  relevanceScore: number;
  excerpt: string;
  position: number;
}

export interface TokenUsage {
  input: number;
  output: number;
  total: number;
}

export interface QueryFilters {
  module?: number;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';
  tags?: string[];
}

export interface QueryRequest {
  query: string;
  userId?: string;
  sessionId?: string;
  filters?: QueryFilters;
  topK?: number;
}

export interface QueryResponse {
  answer: string;
  sources: SourceCitation[];
  confidence: number;
  sessionId: string;
  tokensUsed: TokenUsage;
  filterMessage?: string;
}

export interface StreamChunk {
  chunk?: string;
  done?: boolean;
  sources?: SourceCitation[];
  confidence?: number;
  tokensUsed?: TokenUsage;
  error?: string;
}
```

---

## References

- Spec Key Entities: `specs/003-rag-chatbot-core/spec.md` (lines 166-178)
- Spec Clarifications: `specs/003-rag-chatbot-core/spec.md` (lines 209-221)
- Constitution Data Governance: `.specify/memory/constitution.md` (lines 883-924)
