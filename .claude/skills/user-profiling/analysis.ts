/**
 * Profile analysis and personalization logic
 */

import { UserProfile, ProfileAnalysis, ExperienceLevel, ROSFamiliarity, HardwareAccess, LearningGoal } from './schema';

export function analyzeProfile(profile: UserProfile): ProfileAnalysis {
  // Determine persona based on experience and familiarity
  const persona = generatePersona(profile.experience_level, profile.ros_familiarity, profile.learning_goal);

  // Determine content level
  const content_level = determineContentLevel(profile.experience_level, profile.ros_familiarity);

  // Generate recommended modules
  const recommended_modules = generateRecommendedModules(profile);

  // Estimate completion time
  const estimated_completion_time = estimateCompletionTime(profile);

  // Generate personalization hints
  const personalization_hints = generatePersonalizationHints(profile);

  return {
    persona,
    content_level,
    recommended_modules,
    estimated_completion_time,
    personalization_hints
  };
}

function generatePersona(experience: ExperienceLevel, rosFamiliarity: ROSFamiliarity, goal: LearningGoal): string {
  const experienceMap = {
    beginner: 'Beginner',
    intermediate: 'Intermediate',
    advanced: 'Advanced'
  };

  const goalMap = {
    career: 'Professional',
    research: 'Researcher',
    hobby: 'Hobbyist'
  };

  return `${experienceMap[experience]} ${goalMap[goal]}`;
}

function determineContentLevel(experience: ExperienceLevel, rosFamiliarity: ROSFamiliarity): 'simplified' | 'standard' | 'advanced' {
  // If user is advanced or proficient in ROS, use advanced content
  if (experience === 'advanced' || rosFamiliarity === 'proficient') {
    return 'advanced';
  }

  // If user is intermediate or has basic ROS knowledge, use standard content
  if (experience === 'intermediate' || rosFamiliarity === 'basic') {
    return 'standard';
  }

  // Otherwise, use simplified content
  return 'simplified';
}

function generateRecommendedModules(profile: UserProfile): string[] {
  const modules = [];

  // Core modules everyone gets
  modules.push('ros-fundamentals', 'python-basics');

  // Experience-based modules
  if (profile.experience_level === 'beginner') {
    modules.unshift('programming-foundations');
  } else if (profile.experience_level === 'advanced') {
    modules.push('advanced-algorithms', 'system-architecture');
  }

  // ROS familiarity-based modules
  if (profile.ros_familiarity === 'none') {
    modules.push('ros-introduction', 'ros-concepts');
  } else if (profile.ros_familiarity === 'basic') {
    modules.push('ros-advanced-topics', 'ros-navigation');
  } else {
    modules.push('ros-expert-techniques', 'ros-optimization');
  }

  // Hardware-based modules
  if (profile.hardware_access === 'simulation') {
    modules.push('gazebo-simulation', 'virtual-robotics');
  } else if (profile.hardware_access === 'jetson') {
    modules.push('jetbot-setup', 'edge-computing');
  } else {
    modules.push('hardware-integration', 'sensor-calibration', 'robot-control');
  }

  // Goal-based modules
  if (profile.learning_goal === 'career') {
    modules.push('industry-best-practices', 'resume-building');
  } else if (profile.learning_goal === 'research') {
    modules.push('research-methodology', 'paper-writing');
  }

  // Language preference affects module order
  if (profile.preferred_language === 'cpp') {
    modules.splice(modules.indexOf('python-basics'), 1);
    modules.push('cpp-for-robotics');
  } else if (profile.preferred_language === 'both') {
    modules.push('cpp-for-robotics');
  }

  return modules;
}

function estimateCompletionTime(profile: UserProfile): string {
  let baseWeeks = 12; // Base course length

  // Adjust based on experience
  if (profile.experience_level === 'beginner') {
    baseWeeks += 8;
  } else if (profile.experience_level === 'advanced') {
    baseWeeks -= 4;
  }

  // Adjust based on ROS familiarity
  if (profile.ros_familiarity === 'none') {
    baseWeeks += 6;
  }

  // Adjust based on hardware access
  if (profile.hardware_access === 'full_lab') {
    baseWeeks += 4; // Additional hardware projects
  }

  // Adjust based on goal
  if (profile.learning_goal === 'research') {
    baseWeeks += 8; // Research projects take longer
  } else if (profile.learning_goal === 'hobby') {
    baseWeeks = Math.round(baseWeeks * 0.75); // More flexible pace
  }

  return `${baseWeeks} weeks`;
}

function generatePersonalizationHints(profile: UserProfile) {
  return {
    show_prerequisites: profile.experience_level === 'beginner' || profile.ros_familiarity === 'none',
    include_research_papers: profile.learning_goal === 'research' || profile.experience_level === 'advanced',
    hardware_specific_content: profile.hardware_access === 'simulation' ? 'simulation-only' :
                               profile.hardware_access === 'jetson' ? 'jetson-focused' : 'full-hardware',
    code_language_filter: profile.preferred_language === 'both' ? 'all' : profile.preferred_language
  };
}

export function getSimilarProfiles(profile_hash: string, count: number = 5): Promise<string[]> {
  // This would query the database for profiles with similar hashes
  // For now, return an empty array
  return Promise.resolve([]);
}

export function generateLearningPath(persona: string, modules: string[]): { stage: string; modules: string[] }[] {
  const stages = [
    { stage: 'Foundation', modules: [] },
    { stage: 'Core Skills', modules: [] },
    { stage: 'Advanced Topics', modules: [] },
    { stage: 'Specialization', modules: [] }
  ];

  // Categorize modules into stages (simplified logic)
  const foundationModules = ['programming-foundations', 'python-basics', 'cpp-for-robotics', 'ros-introduction'];
  const coreModules = ['ros-fundamentals', 'ros-concepts', 'gazebo-simulation', 'jetbot-setup'];
  const advancedModules = ['ros-advanced-topics', 'advanced-algorithms', 'edge-computing', 'hardware-integration'];
  const specializationModules = ['industry-best-practices', 'research-methodology', 'ros-expert-techniques'];

  modules.forEach(module => {
    if (foundationModules.includes(module)) {
      stages[0].modules.push(module);
    } else if (coreModules.includes(module)) {
      stages[1].modules.push(module);
    } else if (advancedModules.includes(module)) {
      stages[2].modules.push(module);
    } else if (specializationModules.includes(module)) {
      stages[3].modules.push(module);
    }
  });

  return stages.filter(stage => stage.modules.length > 0);
}