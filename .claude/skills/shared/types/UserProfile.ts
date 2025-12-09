/**
 * Unified User Profile Type Definition
 *
 * This type is used across all skills to ensure consistency in user profile data.
 * Any changes to this type should be reflected in:
 * - Database schema (neon-postgres)
 * - Authentication (better-auth-setup)
 * - Content adaptation (content-adapter)
 * - User profiling (user-profiling)
 */

export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';
export type RosFamiliarity = 'none' | 'basic' | 'proficient';
export type HardwareAccess = 'simulation_only' | 'partial_lab' | 'full_lab';
export type LearningGoal = 'career' | 'research' | 'hobby';
export type PreferredLanguage = 'python' | 'cpp' | 'both';
export type ContentLanguage = 'en' | 'ur';

export interface UserProfile {
  /**
   * User's programming and robotics experience level
   * - beginner: < 1 year coding experience
   * - intermediate: 1-3 years coding experience
   * - advanced: 3+ years coding experience
   */
  experience_level: ExperienceLevel;

  /**
   * User's familiarity with ROS (Robot Operating System)
   * - none: No ROS experience
   * - basic: Completed ROS tutorials
   * - proficient: Built ROS projects
   */
  ros_familiarity: RosFamiliarity;

  /**
   * Hardware resources available to the user
   * - simulation_only: Only Gazebo/Isaac Sim access
   * - partial_lab: Jetson kit with sensors
   * - full_lab: Complete robot lab with multiple robots
   */
  hardware_access: HardwareAccess;

  /**
   * User's primary goal for learning
   * - career: Career advancement in robotics industry
   * - research: Academic research and publications
   * - hobby: Personal interest and exploration
   */
  learning_goal: LearningGoal;

  /**
   * Preferred programming language for code examples
   * - python: Show Python code examples
   * - cpp: Show C++ code examples
   * - both: Show both Python and C++ examples
   */
  preferred_language: PreferredLanguage;

  /**
   * Preferred language for content (text, explanations)
   * - en: English content
   * - ur: Urdu content
   */
  content_language?: ContentLanguage;
}

export interface UserProfileWithMetadata extends UserProfile {
  user_id: string;
  profile_hash: string;
  created_at: Date;
  updated_at: Date;
}

export interface ProfileAnalysis {
  /**
   * User persona based on profile
   * e.g., "Beginner Hobbyist", "Advanced Researcher", "Intermediate Professional"
   */
  persona: string;

  /**
   * Recommended content difficulty level
   */
  content_level: 'simplified' | 'standard' | 'advanced';

  /**
   * Recommended learning modules in order
   */
  recommended_modules: string[];

  /**
   * Estimated time to complete the course
   * e.g., "12 weeks", "16 weeks", "20 weeks"
   */
  estimated_completion_time: string;

  /**
   * Personalization hints for content adaptation
   */
  personalization_hints: {
    show_prerequisites: boolean;
    include_research_papers: boolean;
    hardware_specific_content: 'simulation' | 'jetson' | 'full-lab';
    code_language_filter: PreferredLanguage;
    show_advanced_topics: boolean;
    ros_basics_sidebar: boolean;
  };
}

/**
 * Validation functions
 */
export function isValidExperienceLevel(value: string): value is ExperienceLevel {
  return ['beginner', 'intermediate', 'advanced'].includes(value);
}

export function isValidRosFamiliarity(value: string): value is RosFamiliarity {
  return ['none', 'basic', 'proficient'].includes(value);
}

export function isValidHardwareAccess(value: string): value is HardwareAccess {
  return ['simulation_only', 'partial_lab', 'full_lab'].includes(value);
}

export function isValidLearningGoal(value: string): value is LearningGoal {
  return ['career', 'research', 'hobby'].includes(value);
}

export function isValidPreferredLanguage(value: string): value is PreferredLanguage {
  return ['python', 'cpp', 'both'].includes(value);
}

export function isValidContentLanguage(value: string): value is ContentLanguage {
  return ['en', 'ur'].includes(value);
}

export function validateUserProfile(profile: Partial<UserProfile>): {
  valid: boolean;
  errors: string[];
} {
  const errors: string[] = [];

  if (!profile.experience_level) {
    errors.push('experience_level is required');
  } else if (!isValidExperienceLevel(profile.experience_level)) {
    errors.push(`Invalid experience_level: ${profile.experience_level}`);
  }

  if (!profile.ros_familiarity) {
    errors.push('ros_familiarity is required');
  } else if (!isValidRosFamiliarity(profile.ros_familiarity)) {
    errors.push(`Invalid ros_familiarity: ${profile.ros_familiarity}`);
  }

  if (!profile.hardware_access) {
    errors.push('hardware_access is required');
  } else if (!isValidHardwareAccess(profile.hardware_access)) {
    errors.push(`Invalid hardware_access: ${profile.hardware_access}`);
  }

  if (!profile.learning_goal) {
    errors.push('learning_goal is required');
  } else if (!isValidLearningGoal(profile.learning_goal)) {
    errors.push(`Invalid learning_goal: ${profile.learning_goal}`);
  }

  if (!profile.preferred_language) {
    errors.push('preferred_language is required');
  } else if (!isValidPreferredLanguage(profile.preferred_language)) {
    errors.push(`Invalid preferred_language: ${profile.preferred_language}`);
  }

  if (profile.content_language && !isValidContentLanguage(profile.content_language)) {
    errors.push(`Invalid content_language: ${profile.content_language}`);
  }

  return {
    valid: errors.length === 0,
    errors
  };
}

/**
 * Generate a hash for the user profile (used for caching)
 */
export function generateProfileHash(profile: UserProfile): string {
  const profileString = JSON.stringify({
    experience_level: profile.experience_level,
    ros_familiarity: profile.ros_familiarity,
    hardware_access: profile.hardware_access,
    learning_goal: profile.learning_goal,
    preferred_language: profile.preferred_language,
    content_language: profile.content_language || 'en'
  });

  // Simple hash function (in production, use crypto.createHash)
  let hash = 0;
  for (let i = 0; i < profileString.length; i++) {
    const char = profileString.charCodeAt(i);
    hash = ((hash << 5) - hash) + char;
    hash = hash & hash;
  }
  return Math.abs(hash).toString(16);
}
