/**
 * Authentication TypeScript types for the Physical AI & Humanoid Robotics Textbook.
 * Mirrors backend Pydantic models from data-model.md.
 */

// ============================================
// User Types
// ============================================

export interface User {
  id: string;
  email: string;
  createdAt: string;
  isActive: boolean;
  hasGoogle: boolean;
  profileComplete: boolean;
}

// ============================================
// Profile Types
// ============================================

export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';
export type ROSFamiliarity = 'none' | 'basic' | 'proficient';
export type HardwareAccess = 'simulation_only' | 'jetson_kit' | 'full_robot_lab';
export type LearningGoal = 'career_transition' | 'academic_research' | 'hobby';
export type PreferredLanguage = 'python' | 'cpp' | 'both';

export interface UserProfile {
  id: string;
  userId: string;
  experienceLevel: ExperienceLevel | null;
  rosFamiliarity: ROSFamiliarity | null;
  hardwareAccess: HardwareAccess | null;
  learningGoal: LearningGoal | null;
  preferredLanguage: PreferredLanguage | null;
  isComplete: boolean;
  createdAt: string;
  updatedAt: string;
}

export interface ProfileUpdate {
  experienceLevel?: ExperienceLevel | null;
  rosFamiliarity?: ROSFamiliarity | null;
  hardwareAccess?: HardwareAccess | null;
  learningGoal?: LearningGoal | null;
  preferredLanguage?: PreferredLanguage | null;
}

// ============================================
// Auth State Types
// ============================================

export interface AuthState {
  isAuthenticated: boolean;
  user: User | null;
  profile: UserProfile | null;
  isLoading: boolean;
}

// ============================================
// Request Types
// ============================================

export interface LoginRequest {
  email: string;
  password: string;
}

export interface SignupRequest {
  email: string;
  password: string;
}

// ============================================
// Response Types
// ============================================

export interface AuthResponse {
  user: User;
  profile: UserProfile | null;
}

export interface MessageResponse {
  message: string;
}

export interface RefreshResponse {
  message: string;
  expiresIn: number;
}

// ============================================
// Error Types
// ============================================

export interface ApiError {
  error: {
    code: string;
    message: string;
    correlationId?: string;
    timestamp?: string;
    details?: Array<{
      field: string;
      message: string;
    }>;
    lockedUntil?: string;
    retryAfter?: number;
  };
}

// ============================================
// Session Types
// ============================================

export interface SessionInfo {
  id: string;
  createdAt: string;
  expiresAt: string;
  userAgent: string | null;
  ipAddress: string | null;
  isCurrent: boolean;
}

export interface SessionsResponse {
  sessions: SessionInfo[];
}
