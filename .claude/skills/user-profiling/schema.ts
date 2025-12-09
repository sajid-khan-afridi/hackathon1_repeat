/**
 * Profile Questions Schema and Types
 */

export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';
export type ROSFamiliarity = 'none' | 'basic' | 'proficient';
export type HardwareAccess = 'simulation' | 'jetson' | 'full_lab';
export type LearningGoal = 'career' | 'research' | 'hobby';
export type PreferredLanguage = 'python' | 'cpp' | 'both';

export interface ProfileQuestion {
  field: string;
  type: 'single_select';
  question: string;
  options: Record<string, string>;
}

export interface ProfileResponse {
  field: string;
  value: string;
}

export interface UserProfile {
  user_id: string;
  experience_level: ExperienceLevel;
  ros_familiarity: ROSFamiliarity;
  hardware_access: HardwareAccess;
  learning_goal: LearningGoal;
  preferred_language: PreferredLanguage;
  profile_hash: string;
  created_at: Date;
  updated_at: Date;
}

export interface ProfileAnalysis {
  persona: string;
  content_level: 'simplified' | 'standard' | 'advanced';
  recommended_modules: string[];
  estimated_completion_time: string;
  personalization_hints: {
    show_prerequisites: boolean;
    include_research_papers: boolean;
    hardware_specific_content: string;
    code_language_filter: string;
  };
}

export const PROFILE_QUESTIONS: ProfileQuestion[] = [
  {
    field: 'experience_level',
    type: 'single_select',
    question: 'What is your programming experience level?',
    options: {
      beginner: 'Beginner (< 1 year coding)',
      intermediate: 'Intermediate (1-3 years)',
      advanced: 'Advanced (3+ years)'
    }
  },
  {
    field: 'ros_familiarity',
    type: 'single_select',
    question: 'How familiar are you with ROS (Robot Operating System)?',
    options: {
      none: 'No experience',
      basic: 'Basic (completed tutorials)',
      proficient: 'Proficient (built ROS projects)'
    }
  },
  {
    field: 'hardware_access',
    type: 'single_select',
    question: 'What robotics hardware do you have access to?',
    options: {
      simulation: 'Simulation only (no physical hardware)',
      jetson: 'Jetson kit with sensors',
      full_lab: 'Full robot lab (robot + sensors + workstation)'
    }
  },
  {
    field: 'learning_goal',
    type: 'single_select',
    question: 'What is your primary goal for this course?',
    options: {
      career: 'Career advancement in robotics',
      research: 'Academic research',
      hobby: 'Personal interest / hobby'
    }
  },
  {
    field: 'preferred_language',
    type: 'single_select',
    question: 'Which programming language do you prefer for code examples?',
    options: {
      python: 'Python',
      cpp: 'C++',
      both: 'Both (show all examples)'
    }
  }
];

export const REQUIRED_FIELDS = ['experience_level', 'ros_familiarity', 'hardware_access', 'learning_goal', 'preferred_language'];

export function validateProfileResponse(responses: ProfileResponse[]): { isValid: boolean; errors: string[] } {
  const errors: string[] = [];
  const responseMap = new Map(responses.map(r => [r.field, r.value]));

  // Check all required fields are present
  for (const field of REQUIRED_FIELDS) {
    if (!responseMap.has(field)) {
      errors.push(`Missing required field: ${field}`);
    }
  }

  // Validate field values
  for (const response of responses) {
    const question = PROFILE_QUESTIONS.find(q => q.field === response.field);
    if (!question) {
      errors.push(`Unknown field: ${response.field}`);
      continue;
    }

    if (!Object.keys(question.options).includes(response.value)) {
      errors.push(`Invalid value for ${response.field}: ${response.value}`);
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

export function calculateProfileCompleteness(responses: ProfileResponse[]): number {
  const responseFields = new Set(responses.map(r => r.field));
  const answeredFields = REQUIRED_FIELDS.filter(field => responseFields.has(field));
  return Math.round((answeredFields.length / REQUIRED_FIELDS.length) * 100);
}

export function generateProfileHash(responses: ProfileResponse[]): string {
  const keyFields = ['experience_level', 'ros_familiarity', 'hardware_access'];
  const values = keyFields.map(field => {
    const response = responses.find(r => r.field === field);
    return response?.value || '';
  }).join('|');

  // Simple hash implementation (in production, use crypto)
  let hash = 0;
  for (let i = 0; i < values.length; i++) {
    const char = values.charCodeAt(i);
    hash = ((hash << 5) - hash) + char;
    hash = hash & hash; // Convert to 32-bit integer
  }
  return Math.abs(hash).toString(16);
}