/**
 * Personalization API Client
 * Frontend API client for Phase 4B personalization endpoints
 */

// Production API URL
const PRODUCTION_API_URL = 'https://hackathon1repeat-production.up.railway.app';

// Get API URL from Docusaurus config or fallback
const getApiUrl = (): string => {
  if (typeof window !== 'undefined') {
    // Check if running on production domain (GitHub Pages) - highest priority
    if (window.location.hostname === 'sajid-khan-afridi.github.io') {
      return PRODUCTION_API_URL;
    }

    // Check for Docusaurus config
    const docusaurusConfig = (window as any).__DOCUSAURUS__;
    if (docusaurusConfig?.siteConfig?.customFields?.apiUrl) {
      return docusaurusConfig.siteConfig.customFields.apiUrl;
    }
  }
  // Default to localhost for development
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiUrl();

/**
 * Skill level classification response
 */
export interface SkillLevelResponse {
  skill_level: 'beginner' | 'intermediate' | 'advanced';
  calculated_at: string;
  based_on_profile: {
    experience_level?: string;
    ros_familiarity?: string;
    hardware_access?: string;
    learning_goal?: string;
    preferred_language?: string;
  };
}

/**
 * Chapter progress response
 */
export interface ChapterProgressResponse {
  chapter_id: string;
  status: 'started' | 'completed';
  is_bookmarked: boolean;
  started_at: string;
  completed_at?: string;
}

/**
 * Chapter recommendation response
 */
export interface ChapterRecommendationResponse {
  chapter_id: string;
  relevance_score: number;
  reason: string;
  recommended_at: string;
  title?: string;
  difficulty_level?: 'beginner' | 'intermediate' | 'advanced';
  module_number?: number;
}

/**
 * Recommendations response with metadata
 */
export interface RecommendationsResponse {
  user_id: string;
  recommendations: ChapterRecommendationResponse[];
  generated_at: string;
  from_cache: boolean;
}

/**
 * API Error response
 */
export interface ApiError {
  detail: string;
}

/**
 * Fetch user's skill level classification
 * GET /api/v1/skill-level
 */
export async function getSkillLevel(): Promise<SkillLevelResponse> {
  const response = await fetch(`${API_BASE_URL}/api/v1/skill-level`, {
    method: 'GET',
    credentials: 'include', // Include cookies for Better Auth session
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to fetch skill level: ${response.status}`);
  }

  return response.json();
}

/**
 * Recalculate user's skill level classification (force refresh)
 * POST /api/v1/skill-level
 */
export async function recalculateSkillLevel(): Promise<SkillLevelResponse> {
  const response = await fetch(`${API_BASE_URL}/api/v1/skill-level`, {
    method: 'POST',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to recalculate skill level: ${response.status}`);
  }

  return response.json();
}

/**
 * Get user's chapter progress (with optional filters)
 * GET /api/v1/progress?status=completed&bookmarked=true
 */
export async function getChapterProgress(filters?: {
  status?: 'started' | 'completed';
  bookmarked?: boolean;
}): Promise<ChapterProgressResponse[]> {
  const params = new URLSearchParams();
  if (filters?.status) params.append('status', filters.status);
  if (filters?.bookmarked !== undefined) params.append('bookmarked', filters.bookmarked.toString());

  const url = `${API_BASE_URL}/api/v1/progress${params.toString() ? `?${params.toString()}` : ''}`;

  const response = await fetch(url, {
    method: 'GET',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to fetch chapter progress: ${response.status}`);
  }

  return response.json();
}

/**
 * Mark chapter as started
 * POST /api/v1/progress/start
 */
export async function markChapterStarted(chapterId: string): Promise<ChapterProgressResponse> {
  const response = await fetch(`${API_BASE_URL}/api/v1/progress/start`, {
    method: 'POST',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ chapter_id: chapterId }),
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to mark chapter as started: ${response.status}`);
  }

  return response.json();
}

/**
 * Mark chapter as completed
 * POST /api/v1/progress/complete
 */
export async function markChapterCompleted(chapterId: string): Promise<ChapterProgressResponse> {
  const response = await fetch(`${API_BASE_URL}/api/v1/progress/complete`, {
    method: 'POST',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ chapter_id: chapterId }),
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to mark chapter as completed: ${response.status}`);
  }

  return response.json();
}

/**
 * Toggle chapter bookmark status
 * POST /api/v1/progress/bookmark
 */
export async function toggleChapterBookmark(chapterId: string): Promise<ChapterProgressResponse> {
  const response = await fetch(`${API_BASE_URL}/api/v1/progress/bookmark`, {
    method: 'POST',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ chapter_id: chapterId }),
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to toggle bookmark: ${response.status}`);
  }

  return response.json();
}

/**
 * Get personalized chapter recommendations
 * GET /api/v1/recommendations?force_refresh=false
 */
export async function getRecommendations(
  forceRefresh: boolean = false
): Promise<RecommendationsResponse> {
  const params = new URLSearchParams();
  if (forceRefresh) params.append('force_refresh', 'true');

  const url = `${API_BASE_URL}/api/v1/recommendations${params.toString() ? `?${params.toString()}` : ''}`;

  const response = await fetch(url, {
    method: 'GET',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
    },
  });

  if (!response.ok) {
    const error: ApiError = await response.json();
    throw new Error(error.detail || `Failed to fetch recommendations: ${response.status}`);
  }

  return response.json();
}
