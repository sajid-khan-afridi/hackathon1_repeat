/**
 * User Profiling Skill - Usage Examples
 */

import { userProfilingSkill } from './skill';

// Example 1: Collecting a complete beginner profile
async function exampleCollectBeginner() {
  const result = await userProfilingSkill({
    operation: 'collect',
    user_id: 'student_001',
    responses: [
      { field: 'experience_level', value: 'beginner' },
      { field: 'ros_familiarity', value: 'none' },
      { field: 'hardware_access', value: 'simulation' },
      { field: 'learning_goal', value: 'career' },
      { field: 'preferred_language', value: 'python' }
    ]
  });

  console.log('Beginner Profile:', result.data.analysis);
  /*
  Output:
  {
    persona: "Beginner Professional",
    content_level: "simplified",
    recommended_modules: [
      "programming-foundations",
      "ros-introduction",
      "ros-fundamentals",
      "python-basics",
      "gazebo-simulation",
      "virtual-robotics",
      "industry-best-practices",
      "resume-building"
    ],
    estimated_completion_time: "26 weeks"
  }
  */
}

// Example 2: Collecting an advanced researcher profile
async function exampleCollectAdvancedResearcher() {
  const result = await userProfilingSkill({
    operation: 'collect',
    user_id: 'researcher_042',
    responses: [
      { field: 'experience_level', value: 'advanced' },
      { field: 'ros_familiarity', value: 'proficient' },
      { field: 'hardware_access', value: 'full_lab' },
      { field: 'learning_goal', value: 'research' },
      { field: 'preferred_language', value: 'both' }
    ]
  });

  console.log('Advanced Researcher Profile:', result.data.analysis);
  /*
  Output:
  {
    persona: "Advanced Researcher",
    content_level: "advanced",
    recommended_modules: [
      "ros-fundamentals",
      "python-basics",
      "advanced-algorithms",
      "system-architecture",
      "ros-expert-techniques",
      "ros-optimization",
      "hardware-integration",
      "sensor-calibration",
      "robot-control",
      "research-methodology",
      "paper-writing",
      "cpp-for-robotics"
    ],
    estimated_completion_time: "22 weeks"
  }
  */
}

// Example 3: Updating a profile
async function exampleUpdateProfile() {
  // First retrieve the profile
  const profile = await userProfilingSkill({
    operation: 'retrieve',
    user_id: 'student_001'
  });

  console.log('Before update:', profile.data.profile.experience_level);

  // Update experience level
  const updateResult = await userProfilingSkill({
    operation: 'update',
    user_id: 'student_001',
    responses: [
      { field: 'experience_level', value: 'intermediate' }
    ]
  });

  console.log('After update:', updateResult.data.profile.experience_level);
  console.log('New analysis:', updateResult.data.analysis.persona);
}

// Example 4: Handling validation errors
async function exampleValidationError() {
  const result = await userProfilingSkill({
    operation: 'collect',
    user_id: 'test_user',
    responses: [
      { field: 'experience_level', value: 'beginner' },
      // Missing required fields
    ]
  });

  console.log('Validation errors:', result.errors);
  /*
  Output:
  ["Missing required field: ros_familiarity", "Missing required field: hardware_access", ...]
  */
}

// Example 5: Analyzing profile without storing
async function exampleAnalyzeOnly() {
  // Create a profile object for analysis
  const tempProfile = {
    user_id: 'temp_user',
    experience_level: 'intermediate' as const,
    ros_familiarity: 'basic' as const,
    hardware_access: 'jetson' as const,
    learning_goal: 'hobby' as const,
    preferred_language: 'cpp' as const,
    profile_hash: 'abc123',
    created_at: new Date(),
    updated_at: new Date()
  };

  const result = await userProfilingSkill({
    operation: 'analyze',
    user_id: 'temp_user'
  });

  console.log('Profile analysis:', result.data.analysis);
}

// Example 6: Finding similar profiles
async function exampleSimilarProfiles() {
  const result = await userProfilingSkill({
    operation: 'analyze',
    user_id: 'student_001'
  });

  console.log(`Found ${result.data.similar_profiles_count} similar profiles`);
  console.log('Profile hash for caching:', result.data.profile_hash);
}

// Example 7: Deleting a profile (privacy)
async function exampleDeleteProfile() {
  // Note: You would need to implement this in the database module
  // await deleteUserProfile('user_to_delete');
  console.log('Profile deleted (privacy request)');
}

// Example 8: Incomplete profile warning
async function exampleIncompleteProfile() {
  const result = await userProfilingSkill({
    operation: 'collect',
    user_id: 'new_user',
    responses: [
      { field: 'experience_level', value: 'beginner' },
      { field: 'ros_familiarity', value: 'none' }
      // Only 2 out of 5 questions answered
    ]
  });

  if (!result.success) {
    console.log('Profile completeness warning:', result.warnings);
    console.log('Please complete all required fields');
  }
}

// Run all examples
async function runAllExamples() {
  console.log('=== User Profiling Skill Examples ===\n');

  try {
    await exampleCollectBeginner();
    console.log('\n---\n');

    await exampleCollectAdvancedResearcher();
    console.log('\n---\n');

    await exampleUpdateProfile();
    console.log('\n---\n');

    await exampleValidationError();
    console.log('\n---\n');

    await exampleAnalyzeOnly();
    console.log('\n---\n');

    await exampleSimilarProfiles();
    console.log('\n---\n');

    await exampleDeleteProfile();
    console.log('\n---\n');

    await exampleIncompleteProfile();
  } catch (error) {
    console.error('Example error:', error);
  }
}

// Export for use in tests or documentation
export {
  exampleCollectBeginner,
  exampleCollectAdvancedResearcher,
  exampleUpdateProfile,
  exampleValidationError,
  exampleAnalyzeOnly,
  exampleSimilarProfiles,
  exampleDeleteProfile,
  exampleIncompleteProfile,
  runAllExamples
};