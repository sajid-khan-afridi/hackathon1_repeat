-- Migration: 007_add_github_oauth
-- Description: Add GitHub OAuth support to users table
-- Date: 2025-12-20
-- Feature: 001-user-auth

-- Add github_id column to users table
ALTER TABLE users
ADD COLUMN IF NOT EXISTS github_id VARCHAR(255) UNIQUE;

-- Create index for github_id lookups
CREATE INDEX IF NOT EXISTS idx_users_github_id ON users(github_id) WHERE github_id IS NOT NULL;

-- Update auth method constraint to include GitHub
ALTER TABLE users
DROP CONSTRAINT IF EXISTS users_auth_method_check;

ALTER TABLE users
ADD CONSTRAINT users_auth_method_check CHECK (
    password_hash IS NOT NULL OR google_id IS NOT NULL OR github_id IS NOT NULL
);
