-- Migration: 006_link_sessions_to_users
-- Description: Add foreign key constraint to link chat_sessions to users table
-- Date: 2025-12-17
-- Feature: 001-user-auth

-- Add foreign key constraint to existing user_id column
-- This links chat sessions to authenticated users
DO $$
BEGIN
    -- Only add constraint if it doesn't exist
    IF NOT EXISTS (
        SELECT 1 FROM information_schema.table_constraints
        WHERE constraint_name = 'fk_chat_sessions_user_id'
        AND table_name = 'chat_sessions'
    ) THEN
        ALTER TABLE chat_sessions
            ADD CONSTRAINT fk_chat_sessions_user_id
            FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE SET NULL;
    END IF;
END
$$;
