#!/usr/bin/env node

import crypto from 'crypto';
import { Pool } from 'pg';

/**
 * User Profiling Skill
 * Manages user profile information for content personalization
 */
class UserProfiling {
  constructor(options = {}) {
    this.pool = null;
  }

  /**
   * Connect to database
   */
  async connect() {
    if (this.pool) return;

    const connectionString = process.env.DATABASE_URL;
    if (!connectionString) {
      throw new Error('DATABASE_URL environment variable is required');
    }

    this.pool = new Pool({
      connectionString,
      max: parseInt(process.env.DATABASE_POOL_MAX || '10'),
      ssl: connectionString.includes('neon.tech') ? { rejectUnauthorized: false } : false
    });
  }

  /**
   * Disconnect from database
   */
  async disconnect() {
    if (this.pool) {
      await this.pool.end();
      this.pool = null;
    }
  }

  /**
   * Collect user profile (create or update)
   */
  async collect(userId, profileData) {
    await this.connect();

    try {
      const profileHash = this.generateProfileHash(profileData);

      const result = await this.pool.query(
        `INSERT INTO user_profiles (
          user_id, experience_level, ros_familiarity, hardware_access,
          learning_goal, preferred_language, content_language, profile_hash
        ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
        ON CONFLICT (user_id) DO UPDATE SET
          experience_level = EXCLUDED.experience_level,
          ros_familiarity = EXCLUDED.ros_familiarity,
          hardware_access = EXCLUDED.hardware_access,
          learning_goal = EXCLUDED.learning_goal,
          preferred_language = EXCLUDED.preferred_language,
          content_language = EXCLUDED.content_language,
          profile_hash = EXCLUDED.profile_hash,
          updated_at = NOW()
        RETURNING *`,
        [
          userId,
          profileData.experience_level,
          profileData.ros_familiarity,
          profileData.hardware_access,
          profileData.learning_goal,
          profileData.preferred_language,
          profileData.content_language || 'en',
          profileHash
        ]
      );

      return {
        success: true,
        profile: result.rows[0],
        profile_hash: profileHash
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Retrieve user profile
   */
  async retrieve(userId) {
    await this.connect();

    try {
      const result = await this.pool.query(
        'SELECT * FROM user_profiles WHERE user_id = $1',
        [userId]
      );

      if (result.rows.length === 0) {
        return {
          success: false,
          error: { message: 'Profile not found', code: 'NOT_FOUND' }
        };
      }

      return {
        success: true,
        profile: result.rows[0],
        profile_hash: result.rows[0].profile_hash
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Update user profile
   */
  async update(userId, profileData) {
    return this.collect(userId, profileData);
  }

  /**
   * Analyze user profile
   */
  async analyze(userId) {
    const profileResult = await this.retrieve(userId);

    if (!profileResult.success) {
      return profileResult;
    }

    const profile = profileResult.profile;
    const analysis = this.generateAnalysis(profile);

    return {
      success: true,
      profile: profile,
      analysis,
      profile_hash: profile.profile_hash
    };
  }

  /**
   * Delete user profile
   */
  async delete(userId) {
    await this.connect();

    try {
      const result = await this.pool.query(
        'DELETE FROM user_profiles WHERE user_id = $1 RETURNING id',
        [userId]
      );

      if (result.rows.length === 0) {
        return {
          success: false,
          error: { message: 'Profile not found', code: 'NOT_FOUND' }
        };
      }

      return {
        success: true,
        deleted: true
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Generate profile hash for caching
   */
  generateProfileHash(profileData) {
    const hashInput = [
      profileData.experience_level,
      profileData.ros_familiarity,
      profileData.hardware_access,
      profileData.learning_goal,
      profileData.preferred_language,
      profileData.content_language || 'en'
    ].join('|');

    return crypto.createHash('sha256').update(hashInput).digest('hex').substring(0, 64);
  }

  /**
   * Generate profile analysis
   */
  generateAnalysis(profile) {
    // Determine persona
    const persona = this.determinePersona(profile);

    // Determine content level
    const contentLevel = this.determineContentLevel(profile);

    // Generate recommendations
    const recommendedModules = this.generateModuleRecommendations(profile);

    // Estimate completion time
    const estimatedTime = this.estimateCompletionTime(profile);

    // Generate personalization hints
    const personalizationHints = this.generateHints(profile);

    return {
      persona,
      content_level: contentLevel,
      recommended_modules: recommendedModules,
      estimated_completion_time: estimatedTime,
      personalization_hints: personalizationHints
    };
  }

  /**
   * Determine user persona
   */
  determinePersona(profile) {
    const levelMap = {
      beginner: 'Beginner',
      intermediate: 'Intermediate',
      advanced: 'Advanced'
    };

    const goalMap = {
      career: 'Professional',
      research: 'Researcher',
      hobby: 'Hobbyist'
    };

    const level = levelMap[profile.experience_level] || 'Unknown';
    const goal = goalMap[profile.learning_goal] || 'Learner';

    return `${level} ${goal}`;
  }

  /**
   * Determine content level
   */
  determineContentLevel(profile) {
    if (profile.experience_level === 'beginner' || profile.ros_familiarity === 'none') {
      return 'simplified';
    }
    if (profile.experience_level === 'advanced' && profile.ros_familiarity === 'proficient') {
      return 'advanced';
    }
    return 'standard';
  }

  /**
   * Generate module recommendations
   */
  generateModuleRecommendations(profile) {
    const modules = [];

    // Module 1: ROS Fundamentals (always recommended)
    modules.push('Module 1: ROS 2 Fundamentals');

    // Module 2: Simulation (depends on hardware access)
    if (profile.hardware_access === 'simulation_only' || profile.hardware_access === 'partial_lab') {
      modules.push('Module 2: Isaac Sim Simulation');
    }

    // Module 3: Applications (depends on experience)
    if (profile.experience_level !== 'beginner') {
      modules.push('Module 3: Robotics Applications');
    }

    // Module 4: Advanced (for advanced users)
    if (profile.experience_level === 'advanced' && profile.ros_familiarity === 'proficient') {
      modules.push('Module 4: Advanced Topics');
    }

    return modules;
  }

  /**
   * Estimate completion time
   */
  estimateCompletionTime(profile) {
    let weeks = 12;

    if (profile.experience_level === 'beginner') weeks += 4;
    if (profile.ros_familiarity === 'none') weeks += 2;
    if (profile.experience_level === 'advanced') weeks -= 4;

    return `${Math.max(8, weeks)} weeks`;
  }

  /**
   * Generate personalization hints
   */
  generateHints(profile) {
    return {
      show_prerequisites: profile.experience_level === 'beginner' || profile.ros_familiarity === 'none',
      include_research_papers: profile.learning_goal === 'research' || profile.experience_level === 'advanced',
      hardware_specific_content: profile.hardware_access === 'full_lab' ? 'full-lab' :
                                 profile.hardware_access === 'partial_lab' ? 'jetson' : 'simulation',
      code_language_filter: profile.preferred_language,
      show_advanced_topics: profile.experience_level === 'advanced',
      ros_basics_sidebar: profile.ros_familiarity === 'none' || profile.ros_familiarity === 'basic'
    };
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: user-profiling <operation> --user_id <id> [options]');
    console.error('Operations: collect, retrieve, update, analyze, delete');
    console.error('');
    console.error('Examples:');
    console.error('  user-profiling collect --user_id user123 --experience beginner --ros none');
    console.error('  user-profiling retrieve --user_id user123');
    console.error('  user-profiling analyze --user_id user123');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const profiling = new UserProfiling();

  try {
    let result;
    const userId = options.user_id;

    if (!userId) {
      throw new Error('--user_id is required');
    }

    const profileData = {
      experience_level: options.experience || 'beginner',
      ros_familiarity: options.ros || 'none',
      hardware_access: options.hardware || 'simulation_only',
      learning_goal: options.goal || 'hobby',
      preferred_language: options.language || 'python',
      content_language: options.content_lang || 'en'
    };

    switch (operation) {
      case 'collect':
        result = await profiling.collect(userId, profileData);
        break;

      case 'retrieve':
        result = await profiling.retrieve(userId);
        break;

      case 'update':
        result = await profiling.update(userId, profileData);
        break;

      case 'analyze':
        result = await profiling.analyze(userId);
        break;

      case 'delete':
        result = await profiling.delete(userId);
        break;

      default:
        console.error('Unknown operation:', operation);
        process.exit(1);
    }

    console.log(JSON.stringify(result, null, 2));
  } catch (error) {
    console.error('Error:', error.message);
    process.exit(1);
  }
}

function parseCliArgs(args) {
  const options = {};
  for (let i = 0; i < args.length; i++) {
    if (args[i].startsWith('--')) {
      const key = args[i].slice(2);
      const value = args[i + 1] && !args[i + 1].startsWith('--') ? args[i + 1] : 'true';
      options[key] = value;
      if (value !== 'true') i++;
    }
  }
  return options;
}

export default UserProfiling;
export { UserProfiling };

const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('user-profiling/index.js');
if (isMainModule) {
  main().catch(console.error);
}
