-- Migration: 005_create_profiles_table
-- Description: Create user_profiles table for learning preferences
-- Date: 2025-12-17
-- Feature: 001-user-auth

-- ============================================
-- USER PROFILES TABLE
-- ============================================

CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    experience_level VARCHAR(50) CHECK (
        experience_level IS NULL OR
        experience_level IN ('beginner', 'intermediate', 'advanced')
    ),
    ros_familiarity VARCHAR(50) CHECK (
        ros_familiarity IS NULL OR
        ros_familiarity IN ('none', 'basic', 'proficient')
    ),
    hardware_access VARCHAR(50) CHECK (
        hardware_access IS NULL OR
        hardware_access IN ('simulation_only', 'jetson_kit', 'full_robot_lab')
    ),
    learning_goal VARCHAR(50) CHECK (
        learning_goal IS NULL OR
        learning_goal IN ('career_transition', 'academic_research', 'hobby')
    ),
    preferred_language VARCHAR(20) CHECK (
        preferred_language IS NULL OR
        preferred_language IN ('python', 'cpp', 'both')
    ),
    is_complete BOOLEAN NOT NULL DEFAULT FALSE,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);

-- Trigger for is_complete auto-calculation and updated_at
CREATE OR REPLACE FUNCTION update_profile_completion()
RETURNS TRIGGER AS $$
BEGIN
    NEW.is_complete = (
        NEW.experience_level IS NOT NULL AND
        NEW.ros_familiarity IS NOT NULL AND
        NEW.hardware_access IS NOT NULL AND
        NEW.learning_goal IS NOT NULL AND
        NEW.preferred_language IS NOT NULL
    );
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

DROP TRIGGER IF EXISTS update_user_profiles_completion ON user_profiles;
CREATE TRIGGER update_user_profiles_completion
    BEFORE INSERT OR UPDATE ON user_profiles
    FOR EACH ROW
    EXECUTE FUNCTION update_profile_completion();
