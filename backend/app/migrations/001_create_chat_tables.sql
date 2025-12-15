-- Migration: Create chat tables for RAG chatbot
-- Date: 2025-12-15
-- Feature: 003-rag-chatbot-core

-- Enable UUID extension if not already enabled
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- =============================================================================
-- Chat Sessions Table
-- =============================================================================
CREATE TABLE IF NOT EXISTS chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID DEFAULT NULL,  -- FK to users table (if auth is implemented)
    session_token VARCHAR(64) NOT NULL UNIQUE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_activity_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}',

    -- Constraint: Either user_id OR session_token must be present
    CONSTRAINT valid_identification CHECK (
        user_id IS NOT NULL OR session_token IS NOT NULL
    )
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_token ON chat_sessions(session_token);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_activity ON chat_sessions(last_activity_at);

-- =============================================================================
-- Chat Messages Table
-- =============================================================================
CREATE TABLE IF NOT EXISTS chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (LENGTH(content) > 0 AND LENGTH(content) <= 10000),
    tokens_used JSONB DEFAULT '{}',
    confidence FLOAT CHECK (confidence IS NULL OR (confidence >= 0.0 AND confidence <= 1.0)),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    -- Constraint: confidence only applies to assistant messages
    CONSTRAINT confidence_only_for_assistant CHECK (
        role != 'user' OR confidence IS NULL
    )
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_chat_messages_session ON chat_messages(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_messages_created ON chat_messages(created_at);
CREATE INDEX IF NOT EXISTS idx_chat_messages_role ON chat_messages(role);

-- =============================================================================
-- Source Citations Table
-- =============================================================================
CREATE TABLE IF NOT EXISTS source_citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES chat_messages(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    chapter_title VARCHAR(255) NOT NULL,
    relevance_score FLOAT NOT NULL CHECK (relevance_score >= 0.0 AND relevance_score <= 1.0),
    excerpt TEXT NOT NULL CHECK (LENGTH(excerpt) <= 500),
    position INT NOT NULL CHECK (position >= 1),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_source_citations_message ON source_citations(message_id);
CREATE INDEX IF NOT EXISTS idx_source_citations_chapter ON source_citations(chapter_id);
CREATE INDEX IF NOT EXISTS idx_source_citations_score ON source_citations(relevance_score DESC);

-- =============================================================================
-- Comments for documentation
-- =============================================================================
COMMENT ON TABLE chat_sessions IS 'Stores conversation sessions (anonymous or authenticated)';
COMMENT ON TABLE chat_messages IS 'Stores individual messages in conversations';
COMMENT ON TABLE source_citations IS 'Stores source citations for assistant responses';

COMMENT ON COLUMN chat_sessions.session_token IS 'Generated using secrets.token_urlsafe(48)';
COMMENT ON COLUMN chat_sessions.metadata IS 'JSON metadata: filters, preferences, etc.';
COMMENT ON COLUMN chat_messages.tokens_used IS 'JSON: {input: int, output: int, total: int}';
COMMENT ON COLUMN chat_messages.confidence IS 'Confidence score (0.0-1.0) for assistant messages';
COMMENT ON COLUMN source_citations.position IS 'Position in ranked results (1-indexed)';
