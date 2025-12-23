-- Migration: 008_personalization_schema
-- Description: Add tables for Phase 4B personalization engine
-- Date: 2025-12-22
-- Dependencies: Requires Phase 4A users and user_profiles tables

-- 1. Skill Level Classifications Table
CREATE TABLE IF NOT EXISTS skill_level_classifications (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
  skill_level VARCHAR(20) NOT NULL CHECK (skill_level IN ('beginner', 'intermediate', 'advanced')),
  calculated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  based_on_profile JSONB NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_skill_level_user ON skill_level_classifications(user_id);
CREATE INDEX IF NOT EXISTS idx_skill_level_tier ON skill_level_classifications(skill_level);

-- 2. Chapter Progress Table
CREATE TABLE IF NOT EXISTS chapter_progress (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  chapter_id VARCHAR(255) NOT NULL,
  status VARCHAR(20) NOT NULL CHECK (status IN ('started', 'completed')),
  is_bookmarked BOOLEAN NOT NULL DEFAULT FALSE,
  started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  completed_at TIMESTAMP WITH TIME ZONE,
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  CONSTRAINT unique_user_chapter UNIQUE (user_id, chapter_id),
  CONSTRAINT completed_at_valid CHECK (
    (status = 'started' AND completed_at IS NULL) OR
    (status = 'completed' AND completed_at IS NOT NULL)
  )
);

CREATE INDEX IF NOT EXISTS idx_chapter_progress_user ON chapter_progress(user_id);
CREATE INDEX IF NOT EXISTS idx_chapter_progress_status ON chapter_progress(status);
CREATE INDEX IF NOT EXISTS idx_chapter_progress_chapter ON chapter_progress(chapter_id);
CREATE INDEX IF NOT EXISTS idx_chapter_progress_bookmarked ON chapter_progress(is_bookmarked) WHERE is_bookmarked = TRUE;

-- 3. Chapter Metadata Table
CREATE TABLE IF NOT EXISTS chapter_metadata (
  chapter_id VARCHAR(255) PRIMARY KEY,
  module_number INTEGER NOT NULL CHECK (module_number > 0),
  title VARCHAR(500) NOT NULL,
  difficulty_level VARCHAR(20) NOT NULL CHECK (difficulty_level IN ('beginner', 'intermediate', 'advanced')),
  prerequisites JSONB NOT NULL DEFAULT '[]',
  requires_hardware BOOLEAN NOT NULL DEFAULT FALSE,
  learning_goal_tags JSONB NOT NULL DEFAULT '[]',
  created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_chapter_metadata_difficulty ON chapter_metadata(difficulty_level);
CREATE INDEX IF NOT EXISTS idx_chapter_metadata_module ON chapter_metadata(module_number);
CREATE INDEX IF NOT EXISTS idx_chapter_metadata_hardware ON chapter_metadata(requires_hardware);

-- 4. Populate Initial Chapter Metadata (10 sample chapters)
-- This data should match the MDX chapters created in Phase 2
INSERT INTO chapter_metadata (chapter_id, module_number, title, difficulty_level, prerequisites, requires_hardware, learning_goal_tags) VALUES
  ('module-1/ros-intro', 1, 'Introduction to ROS 2', 'beginner', '[]', FALSE, '["theoretical", "practical"]'),
  ('module-1/linux-basics', 1, 'Linux Basics for Robotics', 'beginner', '[]', FALSE, '["practical"]'),
  ('module-1/python-basics', 1, 'Python Programming for ROS', 'beginner', '[]', FALSE, '["practical"]'),
  ('module-1/gazebo-simulation', 1, 'Gazebo Simulation Setup', 'beginner', '["module-1/linux-basics"]', FALSE, '["practical"]'),
  ('module-2/ros-publishers', 2, 'Creating ROS 2 Publishers', 'intermediate', '["module-1/ros-intro", "module-1/python-basics"]', FALSE, '["practical", "theoretical"]'),
  ('module-2/ros-subscribers', 2, 'Creating ROS 2 Subscribers', 'intermediate', '["module-1/ros-intro", "module-1/python-basics"]', FALSE, '["practical", "theoretical"]'),
  ('module-2/ros-services', 2, 'ROS 2 Services and Clients', 'intermediate', '["module-2/ros-publishers", "module-2/ros-subscribers"]', FALSE, '["practical", "theoretical"]'),
  ('module-3/advanced-control', 3, 'Advanced Robot Control', 'advanced', '["module-2/ros-publishers", "module-2/ros-subscribers"]', TRUE, '["practical", "research"]'),
  ('module-3/inverse-kinematics', 3, 'Inverse Kinematics for Manipulators', 'advanced', '["module-2/ros-services"]', TRUE, '["theoretical", "research"]'),
  ('module-3/isaac-integration', 3, 'NVIDIA Isaac Sim Integration', 'advanced', '["module-1/gazebo-simulation", "module-2/ros-publishers"]', FALSE, '["practical", "research"]')
ON CONFLICT (chapter_id) DO NOTHING;

-- Migration complete
