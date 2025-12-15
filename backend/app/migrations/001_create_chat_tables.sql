-- Migration: Create chat tables
-- Description: Creates tables for chat sessions, messages, and citations
-- Version: 001
-- Date: 2025-12-14

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "pgcrypto";

-- Chat sessions table
CREATE TABLE IF NOT EXISTS chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    session_token VARCHAR(64) NOT NULL UNIQUE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_activity_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}',
    is_active BOOLEAN DEFAULT true,
    message_count INTEGER DEFAULT 0,

    CONSTRAINT valid_identification CHECK (
        user_id IS NOT NULL OR session_token IS NOT NULL
    ),
    CONSTRAINT valid_message_count CHECK (
        message_count >= 0
    )
);

-- Create indexes for chat_sessions
CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_session_token ON chat_sessions(session_token);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_last_activity ON chat_sessions(last_activity_at);
CREATE INDEX IF NOT EXISTS idx_chat_sessions_is_active ON chat_sessions(is_active);

-- Chat messages table
CREATE TABLE IF NOT EXISTS chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    token_usage JSONB,
    metadata JSONB DEFAULT '{}',

    CONSTRAINT valid_role CHECK (role IN ('user', 'assistant'))
);

-- Create indexes for chat_messages
CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id ON chat_messages(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_messages_timestamp ON chat_messages(timestamp);
CREATE INDEX IF NOT EXISTS idx_chat_messages_role ON chat_messages(role);

-- Source citations table
CREATE TABLE IF NOT EXISTS source_citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES chat_messages(id) ON DELETE CASCADE,
    chapter_id INTEGER NOT NULL,
    chapter_title VARCHAR(255) NOT NULL,
    section VARCHAR(255),
    module INTEGER,
    module_title VARCHAR(255),
    page_number INTEGER,
    content TEXT NOT NULL,
    relevance_score DECIMAL(3,2) NOT NULL CHECK (relevance_score >= 0 AND relevance_score <= 1),
    similarity_score DECIMAL(3,2) CHECK (similarity_score >= 0 AND similarity_score <= 1),
    position INTEGER,
    page_url VARCHAR(500),
    tags TEXT[] DEFAULT '{}',
    difficulty VARCHAR(20) CHECK (difficulty IN ('beginner', 'intermediate', 'advanced')),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),

    CONSTRAINT valid_relevance_score CHECK (relevance_score >= 0 AND relevance_score <= 1),
    CONSTRAINT valid_similarity_score CHECK (
        similarity_score IS NULL OR (similarity_score >= 0 AND similarity_score <= 1)
    ),
    CONSTRAINT valid_position CHECK (position > 0),
    CONSTRAINT valid_module CHECK (module >= 1 AND module <= 10)
);

-- Create indexes for source_citations
CREATE INDEX IF NOT EXISTS idx_source_citations_message_id ON source_citations(message_id);
CREATE INDEX IF NOT EXISTS idx_source_citations_chapter_id ON source_citations(chapter_id);
CREATE INDEX IF NOT EXISTS idx_source_citations_module ON source_citations(module);
CREATE INDEX IF NOT EXISTS idx_source_citations_relevance ON source_citations(relevance_score);
CREATE INDEX IF NOT EXISTS idx_source_citations_tags ON source_citations USING GIN(tags);

-- Create trigger to update chat_sessions.last_activity_at
CREATE OR REPLACE FUNCTION update_last_activity()
RETURNS TRIGGER AS $$
BEGIN
    UPDATE chat_sessions
    SET last_activity_at = NEW.timestamp,
        message_count = (
            SELECT COUNT(*)
            FROM chat_messages
            WHERE session_id = NEW.session_id
        )
    WHERE id = NEW.session_id;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Create trigger
DROP TRIGGER IF EXISTS trigger_update_last_activity ON chat_messages;
CREATE TRIGGER trigger_update_last_activity
    AFTER INSERT ON chat_messages
    FOR EACH ROW
    EXECUTE FUNCTION update_last_activity();

-- Create function to clean up old sessions
CREATE OR REPLACE FUNCTION cleanup_old_sessions()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
BEGIN
    DELETE FROM chat_sessions
    WHERE last_activity_at < NOW() - INTERVAL '30 days'
    AND is_active = false;

    GET DIAGNOSTICS deleted_count = ROW_COUNT;
    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Migration: Creates core tables for chat functionality: sessions, messages, and citations