/**
 * Authentication hook for easy access to auth state and methods.
 * Re-exports useAuthContext with additional convenience methods.
 */

import { useAuthContext } from '../context/AuthContext';
import type { LoginRequest, SignupRequest } from '../types/auth';

/**
 * Hook for accessing authentication state and methods.
 *
 * @example
 * ```tsx
 * function MyComponent() {
 *   const { isAuthenticated, user, login, logout } = useAuth();
 *
 *   if (!isAuthenticated) {
 *     return <LoginForm onSubmit={login} />;
 *   }
 *
 *   return (
 *     <div>
 *       <p>Welcome, {user?.email}</p>
 *       <button onClick={logout}>Logout</button>
 *     </div>
 *   );
 * }
 * ```
 */
export function useAuth() {
  const context = useAuthContext();

  return {
    // State
    isAuthenticated: context.isAuthenticated,
    isLoading: context.isLoading,
    user: context.user,
    profile: context.profile,

    // Methods
    login: context.login,
    signup: context.signup,
    logout: context.logout,
    refreshAuth: context.refreshAuth,
    updateProfile: context.updateProfile,

    // Convenience getters
    isProfileComplete: context.profile?.isComplete ?? false,
    userEmail: context.user?.email ?? null,
    userId: context.user?.id ?? null,
  };
}

export default useAuth;
