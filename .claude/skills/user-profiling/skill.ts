/**
 * User Profiling Skill - Main handler
 */

import {
  ProfileResponse,
  UserProfile,
  ProfileAnalysis,
  validateProfileResponse,
  calculateProfileCompleteness,
  generateProfileHash,
  PROFILE_QUESTIONS,
  REQUIRED_FIELDS
} from './schema';
import {
  initUserProfileTable,
  createUserProfile,
  updateUserProfile,
  getUserProfile,
  deleteUserProfile,
  getProfilesByHash
} from './database';
import { analyzeProfile } from './analysis';

export interface SkillInput {
  operation: 'collect' | 'retrieve' | 'update' | 'analyze';
  user_id?: string;
  responses?: ProfileResponse[];
}

export interface SkillOutput {
  success: boolean;
  data?: any;
  errors?: string[];
  warnings?: string[];
}

export async function userProfilingSkill(input: SkillInput): Promise<SkillOutput> {
  try {
    // Initialize database table if needed
    await initUserProfileTable();

    switch (input.operation) {
      case 'collect':
        return await handleCollect(input);
      case 'retrieve':
        return await handleRetrieve(input);
      case 'update':
        return await handleUpdate(input);
      case 'analyze':
        return await handleAnalyze(input);
      default:
        return {
          success: false,
          errors: [`Invalid operation: ${input.operation}`]
        };
    }
  } catch (error) {
    return {
      success: false,
      errors: [`Error executing operation: ${error.message}`]
    };
  }
}

async function handleCollect(input: SkillInput): Promise<SkillOutput> {
  if (!input.responses || input.responses.length === 0) {
    return {
      success: false,
      errors: ['No profile responses provided']
    };
  }

  // Validate responses
  const validation = validateProfileResponse(input.responses);
  if (!validation.isValid) {
    return {
      success: false,
      errors: validation.errors
    };
  }

  // Check if all required fields are present
  const completeness = calculateProfileCompleteness(input.responses);
  if (completeness < 100) {
    const warnings = [`Profile is ${completeness}% complete. Please answer all questions.`];
    return {
      success: false,
      errors: ['Incomplete profile'],
      warnings
    };
  }

  // Generate user_id if not provided
  const user_id = input.user_id || generateUserId();

  // Create profile object
  const responseMap = new Map(input.responses.map(r => [r.field, r.value]));
  const profileHash = generateProfileHash(input.responses);

  const profile: Omit<UserProfile, 'created_at' | 'updated_at'> = {
    user_id,
    experience_level: responseMap.get('experience_level') as any,
    ros_familiarity: responseMap.get('ros_familiarity') as any,
    hardware_access: responseMap.get('hardware_access') as any,
    learning_goal: responseMap.get('learning_goal') as any,
    preferred_language: responseMap.get('preferred_language') as any,
    profile_hash: profileHash
  };

  // Save to database
  const savedProfile = await createUserProfile(profile);

  // Generate analysis
  const analysis = analyzeProfile(savedProfile);

  return {
    success: true,
    data: {
      profile: savedProfile,
      analysis,
      completeness: 100,
      user_id
    }
  };
}

async function handleRetrieve(input: SkillInput): Promise<SkillOutput> {
  if (!input.user_id) {
    return {
      success: false,
      errors: ['user_id is required for retrieve operation']
    };
  }

  const profile = await getUserProfile(input.user_id);
  if (!profile) {
    return {
      success: false,
      errors: ['Profile not found']
    };
  }

  // Convert profile to response format
  const responses: ProfileResponse[] = [
    { field: 'experience_level', value: profile.experience_level },
    { field: 'ros_familiarity', value: profile.ros_familiarity },
    { field: 'hardware_access', value: profile.hardware_access },
    { field: 'learning_goal', value: profile.learning_goal },
    { field: 'preferred_language', value: profile.preferred_language }
  ];

  return {
    success: true,
    data: {
      profile,
      responses,
      completeness: 100
    }
  };
}

async function handleUpdate(input: SkillInput): Promise<SkillOutput> {
  if (!input.user_id) {
    return {
      success: false,
      errors: ['user_id is required for update operation']
    };
  }

  if (!input.responses || input.responses.length === 0) {
    return {
      success: false,
      errors: ['No responses provided for update']
    };
  }

  // Check if profile exists
  const existingProfile = await getUserProfile(input.user_id);
  if (!existingProfile) {
    return {
      success: false,
      errors: ['Profile not found. Use collect operation to create a new profile.']
    };
  }

  // Validate responses
  const validation = validateProfileResponse(input.responses);
  if (!validation.isValid) {
    return {
      success: false,
      errors: validation.errors
    };
  }

  // Update profile
  const updatedProfile = await updateUserProfile(input.user_id, input.responses);

  // Generate new analysis
  const analysis = analyzeProfile(updatedProfile);

  return {
    success: true,
    data: {
      profile: updatedProfile,
      analysis,
      updated: true
    }
  };
}

async function handleAnalyze(input: SkillInput): Promise<SkillOutput> {
  if (!input.user_id) {
    return {
      success: false,
      errors: ['user_id is required for analyze operation']
    };
  }

  const profile = await getUserProfile(input.user_id);
  if (!profile) {
    return {
      success: false,
      errors: ['Profile not found']
    };
  }

  // Generate analysis
  const analysis = analyzeProfile(profile);

  // Find similar profiles
  const similarProfiles = await getProfilesByHash(profile.profile_hash);

  return {
    success: true,
    data: {
      user_id: input.user_id,
      analysis,
      similar_profiles_count: similarProfiles.length,
      profile_hash: profile.profile_hash
    }
  };
}

function generateUserId(): string {
  return 'user_' + Math.random().toString(36).substr(2, 9) + '_' + Date.now();
}

// Export additional utilities
export {
  PROFILE_QUESTIONS,
  REQUIRED_FIELDS,
  validateProfileResponse,
  calculateProfileCompleteness
};