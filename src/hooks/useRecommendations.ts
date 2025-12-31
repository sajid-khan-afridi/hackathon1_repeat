/**
 * useRecommendations Hook
 * React hook for fetching and managing personalized chapter recommendations
 *
 * Implementation: T057 - User Story 3 (Smart Chapter Recommendations)
 */

import { useState, useEffect, useCallback } from 'react';
import {
  getRecommendations,
  ChapterRecommendationResponse,
  RecommendationsResponse,
} from '../services/personalization-api';
import { useAuth } from './useAuth';

export interface UseRecommendationsReturn {
  /**
   * List of recommended chapters (max 3)
   */
  recommendations: ChapterRecommendationResponse[];

  /**
   * When recommendations were generated
   */
  generatedAt: string | null;

  /**
   * Whether recommendations came from cache
   */
  fromCache: boolean;

  /**
   * Loading state for initial fetch
   */
  isLoading: boolean;

  /**
   * Loading state for refresh operation
   */
  isRefreshing: boolean;

  /**
   * Error message if fetch failed
   */
  error: string | null;

  /**
   * Force refresh recommendations (bypass cache)
   */
  forceRefresh: () => Promise<void>;

  /**
   * Manually refresh recommendations from server (uses cache if available)
   */
  refresh: () => Promise<void>;
}

/**
 * Hook to fetch and manage personalized chapter recommendations
 *
 * Features:
 * - Automatic fetch on mount when user is authenticated
 * - Uses server-side caching (1-hour TTL)
 * - Force refresh option to bypass cache
 * - Handles loading and error states
 *
 * @example
 * ```tsx
 * function RecommendationsSection() {
 *   const { recommendations, isLoading, error, forceRefresh } = useRecommendations();
 *
 *   if (isLoading) return <div>Loading recommendations...</div>;
 *   if (error) return <div>Error: {error}</div>;
 *
 *   return (
 *     <div>
 *       <h2>Recommended For You</h2>
 *       {recommendations.map(rec => (
 *         <RecommendationCard key={rec.chapter_id} recommendation={rec} />
 *       ))}
 *       <button onClick={forceRefresh}>Refresh</button>
 *     </div>
 *   );
 * }
 * ```
 */
export function useRecommendations(): UseRecommendationsReturn {
  const { user, isLoading: authLoading } = useAuth();

  const [recommendations, setRecommendations] = useState<ChapterRecommendationResponse[]>([]);
  const [generatedAt, setGeneratedAt] = useState<string | null>(null);
  const [fromCache, setFromCache] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [isRefreshing, setIsRefreshing] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Fetch recommendations from API
   */
  const fetchRecommendations = useCallback(
    async (forceRefreshParam: boolean = false) => {
      if (!user) {
        // User not authenticated - clear state
        setRecommendations([]);
        setGeneratedAt(null);
        setFromCache(false);
        setError(null);
        return;
      }

      // Set appropriate loading state
      if (forceRefreshParam) {
        setIsRefreshing(true);
      } else {
        setIsLoading(true);
      }
      setError(null);

      try {
        const response: RecommendationsResponse = await getRecommendations(forceRefreshParam);
        setRecommendations(response.recommendations);
        setGeneratedAt(response.generated_at);
        setFromCache(response.from_cache);

        console.log(
          `[useRecommendations] Fetched ${response.recommendations.length} recommendations, ` +
            `from_cache=${response.from_cache}, force_refresh=${forceRefreshParam}`
        );
      } catch (err) {
        const errorMessage = err instanceof Error ? err.message : 'Failed to fetch recommendations';
        setError(errorMessage);
        console.error('[useRecommendations] Fetch error:', err);
      } finally {
        if (forceRefreshParam) {
          setIsRefreshing(false);
        } else {
          setIsLoading(false);
        }
      }
    },
    [user]
  );

  /**
   * Force refresh recommendations (bypass cache)
   */
  const forceRefresh = useCallback(async () => {
    if (!user) {
      setError('User must be authenticated to fetch recommendations');
      return;
    }

    await fetchRecommendations(true);
  }, [user, fetchRecommendations]);

  /**
   * Refresh recommendations (uses cache if available)
   */
  const refresh = useCallback(async () => {
    await fetchRecommendations(false);
  }, [fetchRecommendations]);

  /**
   * Fetch recommendations on mount and when user auth state changes
   */
  useEffect(() => {
    if (!authLoading && user) {
      fetchRecommendations(false);
    }
  }, [authLoading, user, fetchRecommendations]);

  return {
    recommendations,
    generatedAt,
    fromCache,
    isLoading: isLoading || authLoading,
    isRefreshing,
    error,
    forceRefresh,
    refresh,
  };
}
