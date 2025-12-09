-- User Profiles Table
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES auth.users(id) ON DELETE CASCADE,
    programming_experience VARCHAR(20) CHECK (programming_experience IN ('Beginner', 'Intermediate', 'Advanced')) NOT NULL,
    ros_familiarity VARCHAR(20) CHECK (ros_familiarity IN ('None', 'Basic', 'Proficient')) NOT NULL,
    hardware_access VARCHAR(20) CHECK (hardware_access IN ('Simulation Only', 'Jetson Kit', 'Full Robot Lab')) NOT NULL,
    learning_goal VARCHAR(20) CHECK (learning_goal IN ('Career', 'Research', 'Hobby')) NOT NULL,
    preferred_code VARCHAR(10) CHECK (preferred_code IN ('Python', 'C++', 'Both')) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    UNIQUE(user_id)
);

-- Indexes for performance
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_profiles_programming_experience ON user_profiles(programming_experience);
CREATE INDEX idx_user_profiles_ros_familiarity ON user_profiles(ros_familiarity);

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Trigger to automatically update updated_at
CREATE TRIGGER update_user_profiles_updated_at
    BEFORE UPDATE ON user_profiles
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();