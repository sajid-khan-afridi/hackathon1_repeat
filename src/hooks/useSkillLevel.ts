/**
 * useSkillLevel Hook
 * React hook for fetching and managing user skill level classification
 */

import { useState, useEffect, useCallback } from 'react';
import {
  getSkillLevel,
  recalculateSkillLevel,
  SkillLevelResponse,
} from '../services/personalization-api';
import { useAuth } from './useAuth';

export type SkillLevel = 'beginner' | 'intermediate' | 'advanced';

export interface UseSkillLevelReturn {
  /**
   * Current skill level (beginner/intermediate/advanced)
   */
  skillLevel: SkillLevel | null;

  /**
   * When the skill level was calculated
   */
  calculatedAt: string | null;

  /**
   * Profile attributes used for classification
   */
  profileSnapshot: SkillLevelResponse['based_on_profile'] | null;

  /**
   * Loading state for initial fetch
   */
  isLoading: boolean;

  /**
   * Loading state for recalculation
   */
  isRecalculating: boolean;

  /**
   * Error message if fetch/recalculation failed
   */
  error: string | null;

  /**
   * Force recalculate skill level (use after profile update)
   */
  recalculate: () => Promise<void>;

  /**
   * Manually refresh skill level from server
   */
  refresh: () => Promise<void>;
}

/**
 * Hook to fetch and manage user skill level classification
 *
 * @example
 * ```tsx
 * function UserProfile() {
 *   const { skillLevel, isLoading, error, recalculate } = useSkillLevel();
 *
 *   if (isLoading) return <div>Loading...</div>;
 *   if (error) return <div>Error: {error}</div>;
 *
 *   return (
 *     <div>
 *       <SkillLevelBadge level={skillLevel} />
 *       <button onClick={recalculate}>Recalculate</button>
 *     </div>
 *   );
 * }
 * ```
 */
export function useSkillLevel(): UseSkillLevelReturn {
  const { user, isLoading: authLoading } = useAuth();

  const [skillLevel, setSkillLevel] = useState<SkillLevel | null>(null);
  const [calculatedAt, setCalculatedAt] = useState<string | null>(null);
  const [profileSnapshot, setProfileSnapshot] = useState<
    SkillLevelResponse['based_on_profile'] | null
  >(null);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [isRecalculating, setIsRecalculating] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Fetch skill level from API
   */
  const fetchSkillLevel = useCallback(async () => {
    if (!user) {
      // User not authenticated - clear state
      setSkillLevel(null);
      setCalculatedAt(null);
      setProfileSnapshot(null);
      setError(null);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await getSkillLevel();
      setSkillLevel(response.skill_level);
      setCalculatedAt(response.calculated_at);
      setProfileSnapshot(response.based_on_profile);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch skill level';
      setError(errorMessage);
      console.error('[useSkillLevel] Fetch error:', err);
    } finally {
      setIsLoading(false);
    }
  }, [user]);

  /**
   * Recalculate skill level (force refresh from server)
   */
  const recalculate = useCallback(async () => {
    if (!user) {
      setError('User must be authenticated to recalculate skill level');
      return;
    }

    setIsRecalculating(true);
    setError(null);

    try {
      const response = await recalculateSkillLevel();
      setSkillLevel(response.skill_level);
      setCalculatedAt(response.calculated_at);
      setProfileSnapshot(response.based_on_profile);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to recalculate skill level';
      setError(errorMessage);
      console.error('[useSkillLevel] Recalculate error:', err);
    } finally {
      setIsRecalculating(false);
    }
  }, [user]);

  /**
   * Refresh skill level (alias for fetchSkillLevel for consistency)
   */
  const refresh = useCallback(async () => {
    await fetchSkillLevel();
  }, [fetchSkillLevel]);

  /**
   * Fetch skill level on mount and when user auth state changes
   */
  useEffect(() => {
    if (!authLoading) {
      fetchSkillLevel();
    }
  }, [authLoading, fetchSkillLevel]);

  return {
    skillLevel,
    calculatedAt,
    profileSnapshot,
    isLoading: isLoading || authLoading,
    isRecalculating,
    error,
    recalculate,
    refresh,
  };
}
