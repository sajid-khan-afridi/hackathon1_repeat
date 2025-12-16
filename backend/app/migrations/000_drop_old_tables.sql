-- Migration: Drop old tables with incompatible schema
-- Date: 2025-12-16
-- Feature: 003-rag-chatbot-core
-- Purpose: Clean up old incompatible tables before recreating with correct schema

-- =============================================================================
-- Drop materialized view if exists (from migration 002)
-- =============================================================================
DROP MATERIALIZED VIEW IF EXISTS session_stats CASCADE;

-- =============================================================================
-- Drop tables in correct order (respecting foreign keys)
-- =============================================================================
DROP TABLE IF EXISTS source_citations CASCADE;
DROP TABLE IF EXISTS chat_messages CASCADE;
DROP TABLE IF EXISTS chat_sessions CASCADE;

-- =============================================================================
-- Drop function if exists
-- =============================================================================
DROP FUNCTION IF EXISTS purge_old_sessions() CASCADE;

-- Success message
DO $$
BEGIN
    RAISE NOTICE 'Successfully dropped old tables and views. Ready for clean recreation.';
END $$;
