/**
 * Authentication API client for the Physical AI & Humanoid Robotics Textbook.
 * Handles all auth-related API calls with cookie-based authentication.
 */

import type {
  AuthResponse,
  LoginRequest,
  SignupRequest,
  ApiError,
  RefreshResponse,
  MessageResponse,
  UserProfile,
  ProfileUpdate,
  User,
} from '../types/auth';

// ============================================
// Case Conversion Utilities
// ============================================

/**
 * Convert snake_case string to camelCase.
 */
function snakeToCamel(str: string): string {
  return str.replace(/_([a-z])/g, (_, letter) => letter.toUpperCase());
}

/**
 * Convert camelCase string to snake_case.
 */
function camelToSnake(str: string): string {
  return str.replace(/[A-Z]/g, (letter) => `_${letter.toLowerCase()}`);
}

/**
 * Recursively convert object keys from snake_case to camelCase.
 */
function convertKeysToCamelCase<T>(obj: unknown): T {
  if (obj === null || obj === undefined) {
    return obj as T;
  }
  if (Array.isArray(obj)) {
    return obj.map((item) => convertKeysToCamelCase(item)) as T;
  }
  if (typeof obj === 'object') {
    const converted: Record<string, unknown> = {};
    for (const [key, value] of Object.entries(obj as Record<string, unknown>)) {
      const camelKey = snakeToCamel(key);
      converted[camelKey] = convertKeysToCamelCase(value);
    }
    return converted as T;
  }
  return obj as T;
}

/**
 * Recursively convert object keys from camelCase to snake_case.
 */
function convertKeysToSnakeCase<T>(obj: unknown): T {
  if (obj === null || obj === undefined) {
    return obj as T;
  }
  if (Array.isArray(obj)) {
    return obj.map((item) => convertKeysToSnakeCase(item)) as T;
  }
  if (typeof obj === 'object') {
    const converted: Record<string, unknown> = {};
    for (const [key, value] of Object.entries(obj as Record<string, unknown>)) {
      const snakeKey = camelToSnake(key);
      converted[snakeKey] = convertKeysToSnakeCase(value);
    }
    return converted as T;
  }
  return obj as T;
}

// Production API URL
const PRODUCTION_API_URL = 'https://hackathon1repeat-production.up.railway.app';

// API base URL - uses Docusaurus config or environment variable
const getApiUrl = (): string => {
  // In Docusaurus, we can access customFields from docusaurus.config.ts
  if (typeof window !== 'undefined') {
    // Check if running on production domain (GitHub Pages)
    if (window.location.hostname === 'sajid-khan-afridi.github.io') {
      return PRODUCTION_API_URL;
    }

    // Check for environment variable first (safe browser check)
    const envUrl = (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) ||
                   (typeof process !== 'undefined' && process.env?.API_URL);
    if (envUrl) return envUrl;

    // Check for Docusaurus config
    const docusaurusConfig = (window as any).__DOCUSAURUS__;
    if (docusaurusConfig?.siteConfig?.customFields?.apiUrl) {
      return docusaurusConfig.siteConfig.customFields.apiUrl;
    }
  }
  // Default to localhost for development
  return 'http://localhost:8000';
};

const API_URL = getApiUrl();

/**
 * Custom error class for API errors.
 */
export class AuthApiError extends Error {
  code: string;
  details?: Array<{ field: string; message: string }>;
  lockedUntil?: string;
  retryAfter?: number;

  constructor(error: ApiError['error']) {
    super(error.message);
    this.name = 'AuthApiError';
    this.code = error.code;
    this.details = error.details;
    this.lockedUntil = error.lockedUntil;
    this.retryAfter = error.retryAfter;
  }
}

/**
 * Track if we're currently refreshing to prevent multiple simultaneous refresh attempts.
 */
let isRefreshing = false;
let refreshPromise: Promise<void> | null = null;

/**
 * Get CSRF token from cookie for CSRF protection.
 * The CSRF token is set by the backend and must be included in state-changing requests.
 */
function getCsrfToken(): string | null {
  if (typeof document === 'undefined') return null;

  const name = 'csrf_token=';
  const decodedCookie = decodeURIComponent(document.cookie);
  const cookies = decodedCookie.split(';');

  for (let cookie of cookies) {
    cookie = cookie.trim();
    if (cookie.startsWith(name)) {
      return cookie.substring(name.length);
    }
  }
  return null;
}

/**
 * Make an authenticated API request with automatic token refresh on 401.
 */
async function apiRequest<T>(
  endpoint: string,
  options: RequestInit = {},
  isRetry: boolean = false
): Promise<T> {
  const url = `${API_URL}${endpoint}`;

  // Get CSRF token for state-changing requests (CSRF protection - FR-027)
  const csrfToken = getCsrfToken();
  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
    ...(options.headers as Record<string, string>),
  };

  // Include CSRF token for POST, PUT, DELETE, PATCH requests
  if (csrfToken && options.method && ['POST', 'PUT', 'DELETE', 'PATCH'].includes(options.method.toUpperCase())) {
    headers['X-CSRF-Token'] = csrfToken;
  }

  const response = await fetch(url, {
    ...options,
    credentials: 'include', // Include cookies in requests
    headers,
  });

  const rawData = await response.json();
  // Convert snake_case keys to camelCase for frontend compatibility
  const data = convertKeysToCamelCase<Record<string, unknown>>(rawData);

  if (!response.ok) {
    // Handle 401 Unauthorized - attempt token refresh
    if (response.status === 401 && !isRetry && endpoint !== '/auth/refresh') {
      // If already refreshing, wait for that to complete
      if (isRefreshing && refreshPromise) {
        try {
          await refreshPromise;
          // Retry the original request after refresh completes
          return apiRequest<T>(endpoint, options, true);
        } catch (refreshError) {
          // Refresh failed, throw original error
          if (data.error) {
            throw new AuthApiError(data.error as ApiError['error']);
          }
          throw new Error((data.message as string) || 'An error occurred');
        }
      }

      // Start refresh process
      isRefreshing = true;
      refreshPromise = (async () => {
        try {
          await fetch(`${API_URL}/auth/refresh`, {
            method: 'POST',
            credentials: 'include',
            headers: {
              'Content-Type': 'application/json',
            },
          });
        } finally {
          isRefreshing = false;
          refreshPromise = null;
        }
      })();

      try {
        await refreshPromise;
        // Retry the original request after successful refresh
        return apiRequest<T>(endpoint, options, true);
      } catch (refreshError) {
        // Refresh failed, throw original error
        if (data.error) {
          throw new AuthApiError(data.error as ApiError['error']);
        }
        throw new Error((data.message as string) || 'An error occurred');
      }
    }

    if (data.error) {
      throw new AuthApiError(data.error as ApiError['error']);
    }
    throw new Error((data.message as string) || 'An error occurred');
  }

  return data as T;
}

/**
 * Sign up a new user with email and password.
 */
export async function signup(data: SignupRequest): Promise<AuthResponse> {
  return apiRequest<AuthResponse>('/auth/signup', {
    method: 'POST',
    body: JSON.stringify(data),
  });
}

/**
 * Log in with email and password.
 */
export async function login(data: LoginRequest): Promise<AuthResponse> {
  return apiRequest<AuthResponse>('/auth/login', {
    method: 'POST',
    body: JSON.stringify(data),
  });
}

/**
 * Log out the current user.
 */
export async function logout(): Promise<MessageResponse> {
  return apiRequest<MessageResponse>('/auth/logout', {
    method: 'POST',
  });
}

/**
 * Refresh the access token using the refresh token cookie.
 */
export async function refreshToken(): Promise<RefreshResponse> {
  return apiRequest<RefreshResponse>('/auth/refresh', {
    method: 'POST',
  });
}

/**
 * Get the current authenticated user's information.
 */
export async function getCurrentUser(): Promise<AuthResponse> {
  return apiRequest<AuthResponse>('/auth/me', {
    method: 'GET',
  });
}

/**
 * Check if the user is authenticated by calling /auth/me.
 * Returns null if not authenticated.
 */
export async function checkAuth(): Promise<AuthResponse | null> {
  try {
    return await getCurrentUser();
  } catch (error) {
    // Not authenticated or token expired
    return null;
  }
}

/**
 * Attempt to refresh token and retry the original request.
 * Used by interceptor pattern for handling 401 errors.
 */
export async function withTokenRefresh<T>(
  request: () => Promise<T>
): Promise<T> {
  try {
    return await request();
  } catch (error) {
    if (error instanceof AuthApiError && error.code === 'INVALID_TOKEN') {
      // Try to refresh token
      try {
        await refreshToken();
        // Retry original request
        return await request();
      } catch (refreshError) {
        // Refresh failed, propagate original error
        throw error;
      }
    }
    throw error;
  }
}

/**
 * Get the current user's profile.
 */
export async function getProfile(): Promise<UserProfile> {
  return apiRequest<UserProfile>('/users/profile', {
    method: 'GET',
  });
}

/**
 * Create a new profile for the current user.
 */
export async function createProfile(data: ProfileUpdate): Promise<UserProfile> {
  // Convert camelCase keys to snake_case for API compatibility
  const snakeCaseData = convertKeysToSnakeCase(data);
  return apiRequest<UserProfile>('/users/profile', {
    method: 'POST',
    body: JSON.stringify(snakeCaseData),
  });
}

/**
 * Update the current user's profile.
 */
export async function updateProfile(data: ProfileUpdate): Promise<UserProfile> {
  // Convert camelCase keys to snake_case for API compatibility
  const snakeCaseData = convertKeysToSnakeCase(data);
  return apiRequest<UserProfile>('/users/profile', {
    method: 'PUT',
    body: JSON.stringify(snakeCaseData),
  });
}

/**
 * Skip the profile wizard by creating an empty profile.
 */
export async function skipProfile(): Promise<UserProfile> {
  return apiRequest<UserProfile>('/users/profile/skip', {
    method: 'POST',
  });
}
