-- Migration: Add automatic purge for old sessions
-- Date: 2025-12-15
-- Feature: 003-rag-chatbot-core
-- Purpose: Auto-delete sessions older than retention period (30 days)

-- =============================================================================
-- Function to purge old sessions
-- =============================================================================
CREATE OR REPLACE FUNCTION purge_old_sessions()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
    retention_days INTEGER := 30;  -- Default retention period
BEGIN
    -- Delete sessions older than retention period
    DELETE FROM chat_sessions
    WHERE last_activity_at < (NOW() - INTERVAL '1 day' * retention_days);

    GET DIAGNOSTICS deleted_count = ROW_COUNT;

    -- Log the purge operation
    RAISE NOTICE 'Purged % old chat sessions (older than % days)', deleted_count, retention_days;

    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- =============================================================================
-- Create scheduled job extension (if available)
-- =============================================================================
-- Note: This requires pg_cron extension which may not be available on all platforms
-- For manual execution or alternative scheduling (cron, celery), use:
-- SELECT purge_old_sessions();

COMMENT ON FUNCTION purge_old_sessions IS
'Deletes chat sessions with last_activity_at older than 30 days. Related messages and citations are cascade deleted.';

-- =============================================================================
-- Alternative: Materialized view for session statistics
-- =============================================================================
CREATE MATERIALIZED VIEW IF NOT EXISTS session_stats AS
SELECT
    DATE(created_at) as date,
    COUNT(*) as sessions_created,
    COUNT(DISTINCT user_id) as unique_users,
    AVG(EXTRACT(EPOCH FROM (last_activity_at - created_at))) as avg_session_duration_seconds
FROM chat_sessions
GROUP BY DATE(created_at)
ORDER BY date DESC;

CREATE UNIQUE INDEX IF NOT EXISTS idx_session_stats_date ON session_stats(date);

COMMENT ON MATERIALIZED VIEW session_stats IS
'Daily session statistics. Refresh with: REFRESH MATERIALIZED VIEW CONCURRENTLY session_stats;';
