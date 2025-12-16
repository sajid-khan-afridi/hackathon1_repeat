-- Migration: Add confidence column to chat_messages table
-- Date: 2025-12-16
-- Feature: 003-rag-chatbot-core
-- Purpose: Add confidence score column for assistant message responses

-- =============================================================================
-- Add confidence column if it doesn't exist
-- =============================================================================
DO $$
BEGIN
    -- Check if confidence column exists, if not add it
    IF NOT EXISTS (
        SELECT 1
        FROM information_schema.columns
        WHERE table_name = 'chat_messages'
        AND column_name = 'confidence'
    ) THEN
        ALTER TABLE chat_messages
        ADD COLUMN confidence FLOAT CHECK (confidence IS NULL OR (confidence >= 0.0 AND confidence <= 1.0));

        -- Add comment
        COMMENT ON COLUMN chat_messages.confidence IS 'Confidence score (0.0-1.0) for assistant messages';

        RAISE NOTICE 'Added confidence column to chat_messages table';
    ELSE
        RAISE NOTICE 'Confidence column already exists, skipping';
    END IF;
END $$;
