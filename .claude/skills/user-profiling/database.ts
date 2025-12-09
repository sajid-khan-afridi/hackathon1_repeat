/**
 * Database utilities for user profile storage
 */

import { Pool } from 'pg';
import { UserProfile, ProfileResponse } from './schema';

// Database connection (use environment variables in production)
const pool = new Pool({
  connectionString: process.env.DATABASE_URL || 'postgresql://localhost:5432/robotics_learning',
  ssl: process.env.NODE_ENV === 'production' ? { rejectUnauthorized: false } : false
});

export async function initUserProfileTable(): Promise<void> {
  const query = `
    CREATE TABLE IF NOT EXISTS user_profiles (
      user_id VARCHAR(255) PRIMARY KEY,
      experience_level VARCHAR(20) NOT NULL,
      ros_familiarity VARCHAR(20) NOT NULL,
      hardware_access VARCHAR(20) NOT NULL,
      learning_goal VARCHAR(20) NOT NULL,
      preferred_language VARCHAR(20) NOT NULL,
      profile_hash VARCHAR(64) NOT NULL,
      created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
      updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
    );

    CREATE INDEX IF NOT EXISTS idx_profile_hash ON user_profiles(profile_hash);
    CREATE INDEX IF NOT EXISTS idx_experience_level ON user_profiles(experience_level);
    CREATE INDEX IF NOT EXISTS idx_ros_familiarity ON user_profiles(ros_familiarity);
    CREATE INDEX IF NOT EXISTS idx_hardware_access ON user_profiles(hardware_access);
  `;

  await pool.query(query);
}

export async function createUserProfile(profile: Omit<UserProfile, 'created_at' | 'updated_at'>): Promise<UserProfile> {
  const query = `
    INSERT INTO user_profiles (user_id, experience_level, ros_familiarity, hardware_access, learning_goal, preferred_language, profile_hash)
    VALUES ($1, $2, $3, $4, $5, $6, $7)
    RETURNING *;
  `;

  const values = [
    profile.user_id,
    profile.experience_level,
    profile.ros_familiarity,
    profile.hardware_access,
    profile.learning_goal,
    profile.preferred_language,
    profile.profile_hash
  ];

  const result = await pool.query(query, values);
  return result.rows[0];
}

export async function updateUserProfile(user_id: string, responses: ProfileResponse[]): Promise<UserProfile> {
  // Build dynamic update query
  const updateFields = responses.map(r => `${r.field} = '${r.value}'`);
  updateFields.push('updated_at = CURRENT_TIMESTAMP');

  const query = `
    UPDATE user_profiles
    SET ${updateFields.join(', ')}
    WHERE user_id = $1
    RETURNING *;
  `;

  const result = await pool.query(query, [user_id]);
  return result.rows[0];
}

export async function getUserProfile(user_id: string): Promise<UserProfile | null> {
  const query = 'SELECT * FROM user_profiles WHERE user_id = $1';
  const result = await pool.query(query, [user_id]);
  return result.rows[0] || null;
}

export async function deleteUserProfile(user_id: string): Promise<boolean> {
  const query = 'DELETE FROM user_profiles WHERE user_id = $1';
  const result = await pool.query(query, [user_id]);
  return result.rowCount > 0;
}

export async function getProfilesByHash(profile_hash: string): Promise<UserProfile[]> {
  const query = 'SELECT * FROM user_profiles WHERE profile_hash = $1';
  const result = await pool.query(query, [profile_hash]);
  return result.rows;
}

export async function getAllProfiles(): Promise<UserProfile[]> {
  const query = 'SELECT * FROM user_profiles ORDER BY created_at DESC';
  const result = await pool.query(query);
  return result.rows;
}

export async function closeDatabaseConnection(): Promise<void> {
  await pool.end();
}