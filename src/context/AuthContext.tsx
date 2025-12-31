/**
 * Authentication context provider for the Physical AI & Humanoid Robotics Textbook.
 * Manages authentication state and provides auth methods to components.
 */

import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  ReactNode,
} from 'react';
import type { AuthState, User, UserProfile, LoginRequest, SignupRequest } from '../types/auth';
import * as authApi from '../services/authApi';

// ============================================
// Context Types
// ============================================

interface AuthContextValue extends AuthState {
  login: (data: LoginRequest) => Promise<void>;
  signup: (data: SignupRequest) => Promise<void>;
  logout: () => Promise<void>;
  refreshAuth: () => Promise<void>;
  updateProfile: (profile: UserProfile) => void;
}

// ============================================
// Context Creation
// ============================================

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

// ============================================
// Provider Component
// ============================================

interface AuthProviderProps {
  children: ReactNode;
}

export function AuthProvider({ children }: AuthProviderProps): React.ReactElement {
  const [state, setState] = useState<AuthState>({
    isAuthenticated: false,
    user: null,
    profile: null,
    isLoading: true,
  });

  // Check auth status on mount
  useEffect(() => {
    const initAuth = async () => {
      try {
        const response = await authApi.checkAuth();
        if (response) {
          setState({
            isAuthenticated: true,
            user: response.user,
            profile: response.profile,
            isLoading: false,
          });
        } else {
          setState({
            isAuthenticated: false,
            user: null,
            profile: null,
            isLoading: false,
          });
        }
      } catch (error) {
        console.error('Auth initialization error:', error);
        setState({
          isAuthenticated: false,
          user: null,
          profile: null,
          isLoading: false,
        });
      }
    };

    initAuth();
  }, []);

  // Login handler
  const login = useCallback(async (data: LoginRequest) => {
    setState((prev) => ({ ...prev, isLoading: true }));
    try {
      const response = await authApi.login(data);
      setState({
        isAuthenticated: true,
        user: response.user,
        profile: response.profile,
        isLoading: false,
      });
    } catch (error) {
      setState((prev) => ({ ...prev, isLoading: false }));
      throw error;
    }
  }, []);

  // Signup handler
  const signup = useCallback(async (data: SignupRequest) => {
    setState((prev) => ({ ...prev, isLoading: true }));
    try {
      const response = await authApi.signup(data);
      setState({
        isAuthenticated: true,
        user: response.user,
        profile: response.profile,
        isLoading: false,
      });
    } catch (error) {
      setState((prev) => ({ ...prev, isLoading: false }));
      throw error;
    }
  }, []);

  // Logout handler
  const logout = useCallback(async () => {
    try {
      await authApi.logout();
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      // Clear state regardless of API result
      setState({
        isAuthenticated: false,
        user: null,
        profile: null,
        isLoading: false,
      });
    }
  }, []);

  // Refresh auth state
  const refreshAuth = useCallback(async () => {
    try {
      const response = await authApi.getCurrentUser();
      setState({
        isAuthenticated: true,
        user: response.user,
        profile: response.profile,
        isLoading: false,
      });
    } catch (error) {
      // Try token refresh
      try {
        await authApi.refreshToken();
        const response = await authApi.getCurrentUser();
        setState({
          isAuthenticated: true,
          user: response.user,
          profile: response.profile,
          isLoading: false,
        });
      } catch (refreshError) {
        // Token refresh failed, user is logged out
        setState({
          isAuthenticated: false,
          user: null,
          profile: null,
          isLoading: false,
        });
      }
    }
  }, []);

  // Update profile in state (called after profile update)
  const updateProfile = useCallback((profile: UserProfile) => {
    setState((prev) => ({
      ...prev,
      profile,
      user: prev.user ? { ...prev.user, profileComplete: profile.isComplete } : null,
    }));
  }, []);

  const value: AuthContextValue = {
    ...state,
    login,
    signup,
    logout,
    refreshAuth,
    updateProfile,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

// ============================================
// Hook
// ============================================

export function useAuthContext(): AuthContextValue {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuthContext must be used within an AuthProvider');
  }
  return context;
}

export default AuthContext;
