-- Migration: Add auto-purge functionality
-- Description: Adds scheduled job for automatic cleanup of old sessions
-- Version: 002
-- Date: 2025-12-14

-- Create a table for scheduled jobs
CREATE TABLE IF NOT EXISTS scheduled_jobs (
    id SERIAL PRIMARY KEY,
    job_name VARCHAR(100) NOT NULL UNIQUE,
    job_type VARCHAR(50) NOT NULL,
    schedule_expression VARCHAR(100) NOT NULL,
    last_run_at TIMESTAMPTZ,
    is_active BOOLEAN DEFAULT true,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    metadata JSONB DEFAULT '{}'
);

-- Insert the cleanup job
INSERT INTO scheduled_jobs (job_name, job_type, schedule_expression, metadata)
VALUES (
    'cleanup_old_sessions',
    'function_call',
    '0 2 * * *',  -- Daily at 2 AM
    '{"function_name": "cleanup_old_sessions", "retention_days": 30}'
) ON CONFLICT (job_name) DO NOTHING;

-- Create a more efficient batch cleanup function
CREATE OR REPLACE FUNCTION batch_cleanup_old_sessions(
    batch_size INTEGER DEFAULT 1000,
    retention_days INTEGER DEFAULT 30
)
RETURNS TABLE(batch_deleted INTEGER, total_remaining INTEGER) AS $$
DECLARE
    batch_count INTEGER;
    total_count INTEGER;
BEGIN
    -- Delete in batches to avoid long-running transactions
    WITH deleted AS (
        DELETE FROM chat_sessions
        WHERE ctid IN (
            SELECT ctid
            FROM chat_sessions
            WHERE last_activity_at < NOW() - INTERVAL '1 day' * retention_days
            AND is_active = false
            LIMIT batch_size
        )
        RETURNING id
    )
    SELECT COUNT(*) INTO batch_count FROM deleted;

    -- Count remaining sessions to clean
    SELECT COUNT(*) INTO total_count
    FROM chat_sessions
    WHERE last_activity_at < NOW() - INTERVAL '1 day' * retention_days
    AND is_active = false;

    RETURN QUERY SELECT batch_count, total_count;
END;
$$ LANGUAGE plpgsql;

-- Create a view for session statistics
CREATE OR REPLACE VIEW session_statistics AS
SELECT
    COUNT(*) as total_sessions,
    COUNT(CASE WHEN is_active = true THEN 1 END) as active_sessions,
    COUNT(CASE WHEN user_id IS NOT NULL THEN 1 END) as authenticated_sessions,
    COUNT(CASE WHEN user_id IS NULL THEN 1 END) as anonymous_sessions,
    AVG(message_count) as avg_messages_per_session,
    MAX(message_count) as max_messages_in_session,
    AVG(EXTRACT(EPOCH FROM (last_activity_at - created_at))/60) as avg_session_duration_minutes,
    COUNT(CASE WHEN last_activity_at < NOW() - INTERVAL '7 days' THEN 1 END) as inactive_7_days,
    COUNT(CASE WHEN last_activity_at < NOW() - INTERVAL '30 days' THEN 1 END) as inactive_30_days
FROM chat_sessions;

-- Create a function to get detailed session stats
CREATE OR REPLACE FUNCTION get_detailed_session_stats(
    days_back INTEGER DEFAULT 7
)
RETURNS TABLE(
    date_bucket DATE,
    new_sessions BIGINT,
    active_sessions BIGINT,
    total_messages BIGINT,
    avg_messages_per_session DECIMAL
) AS $$
BEGIN
    RETURN QUERY
    SELECT
        DATE(created_at) as date_bucket,
        COUNT(*) as new_sessions,
        COUNT(CASE WHEN last_activity_at >= NOW() - INTERVAL '1 day' * days_back THEN 1 END) as active_sessions,
        COALESCE(message_totals.total_messages, 0) as total_messages,
        CASE
            WHEN COUNT(*) > 0 THEN
                ROUND(COALESCE(message_totals.total_messages, 0)::DECIMAL / COUNT(*)::DECIMAL, 2)
            ELSE 0
        END as avg_messages_per_session
    FROM chat_sessions cs
    LEFT JOIN (
        SELECT
            session_id,
            COUNT(*) as total_messages
        FROM chat_messages
        WHERE timestamp >= NOW() - INTERVAL '1 day' * days_back
        GROUP BY session_id
    ) message_totals ON cs.id = message_totals.session_id
    WHERE cs.created_at >= NOW() - INTERVAL '1 day' * days_back
    GROUP BY DATE(created_at), message_totals.total_messages
    ORDER BY date_bucket DESC;
END;
$$ LANGUAGE plpgsql;

-- Create indexes for better query performance
CREATE INDEX IF NOT EXISTS idx_chat_sessions_cleanup_candidate
ON chat_sessions(last_activity_at, is_active)
WHERE is_active = false;

CREATE INDEX IF NOT EXISTS idx_chat_messages_session_timestamp
ON chat_messages(session_id, timestamp DESC);

-- Add comments
COMMENT ON TABLE scheduled_jobs IS 'Tracks scheduled maintenance jobs and their execution';
COMMENT ON FUNCTION batch_cleanup_old_sessions IS 'Efficiently deletes old sessions in batches';
COMMENT ON VIEW session_statistics IS 'Provides aggregated statistics about chat sessions';
COMMENT ON FUNCTION get_detailed_session_stats IS 'Returns detailed session metrics for a time period';

-- Migration: Adds auto-purge functionality and session statistics