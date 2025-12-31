/**
 * Chapter progress tracking hook with automatic timer logic.
 * Tracks when users start, complete, and bookmark chapters.
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { useAuth } from './useAuth';
import type {
  ChapterProgressResponse,
  ChapterProgressMarkStarted,
  ChapterProgressMarkCompleted,
  ChapterProgressToggleBookmark,
} from '../types/progress';

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

// Auto-complete threshold: mark chapter as completed after reading for 5 minutes
const AUTO_COMPLETE_THRESHOLD_MS = 5 * 60 * 1000; // 5 minutes

// Scroll depth threshold: consider chapter "read" if scrolled > 80%
const SCROLL_DEPTH_THRESHOLD = 0.8;

interface UseChapterProgressOptions {
  chapterId: string;
  autoMarkStarted?: boolean; // Auto-mark chapter as started on mount (default: true)
  autoMarkCompleted?: boolean; // Auto-mark as completed based on time + scroll (default: false)
  autoCompleteThresholdMs?: number; // Custom threshold for auto-completion
}

interface UseChapterProgressReturn {
  // State
  progress: ChapterProgressResponse | null;
  isLoading: boolean;
  error: string | null;
  timeSpent: number; // Time spent in milliseconds
  scrollDepth: number; // Scroll depth as percentage (0-1)

  // Actions
  markStarted: () => Promise<void>;
  markCompleted: () => Promise<void>;
  toggleBookmark: () => Promise<void>;

  // Utilities
  isStarted: boolean;
  isCompleted: boolean;
  isBookmarked: boolean;
  shouldAutoComplete: boolean; // True if time + scroll thresholds met
}

/**
 * Hook for tracking chapter progress with automatic timer and scroll tracking.
 *
 * @example
 * ```tsx
 * function ChapterPage({ chapterId }: { chapterId: string }) {
 *   const {
 *     progress,
 *     timeSpent,
 *     scrollDepth,
 *     markCompleted,
 *     toggleBookmark,
 *     isBookmarked,
 *   } = useChapterProgress({ chapterId, autoMarkStarted: true });
 *
 *   return (
 *     <div>
 *       <p>Time spent: {Math.floor(timeSpent / 1000)}s</p>
 *       <p>Scroll depth: {Math.floor(scrollDepth * 100)}%</p>
 *       <button onClick={toggleBookmark}>
 *         {isBookmarked ? 'Unbookmark' : 'Bookmark'}
 *       </button>
 *       <button onClick={markCompleted}>Mark as Complete</button>
 *     </div>
 *   );
 * }
 * ```
 */
export function useChapterProgress({
  chapterId,
  autoMarkStarted = true,
  autoMarkCompleted = false,
  autoCompleteThresholdMs = AUTO_COMPLETE_THRESHOLD_MS,
}: UseChapterProgressOptions): UseChapterProgressReturn {
  const { isAuthenticated, userId } = useAuth();

  const [progress, setProgress] = useState<ChapterProgressResponse | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  // Timer tracking
  const [timeSpent, setTimeSpent] = useState<number>(0);
  const timerRef = useRef<number | null>(null);
  const startTimeRef = useRef<number | null>(null);

  // Scroll tracking
  const [scrollDepth, setScrollDepth] = useState<number>(0);

  /**
   * Fetch current progress for the chapter.
   */
  const fetchProgress = useCallback(async () => {
    if (!isAuthenticated || !userId) return;

    try {
      setIsLoading(true);
      setError(null);

      const response = await fetch(`${API_BASE_URL}/api/v1/progress?status=all`, {
        credentials: 'include',
      });

      if (!response.ok) {
        throw new Error(`Failed to fetch progress: ${response.statusText}`);
      }

      const progressList: ChapterProgressResponse[] = await response.json();
      const chapterProgress = progressList.find((p) => p.chapter_id === chapterId);

      setProgress(chapterProgress || null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setIsLoading(false);
    }
  }, [isAuthenticated, userId, chapterId]);

  /**
   * Mark chapter as started.
   */
  const markStarted = useCallback(async () => {
    if (!isAuthenticated || progress?.status === 'completed') return;

    try {
      setError(null);

      const body: ChapterProgressMarkStarted = { chapter_id: chapterId };

      const response = await fetch(`${API_BASE_URL}/api/v1/progress/start`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        throw new Error(`Failed to mark chapter as started: ${response.statusText}`);
      }

      const updated: ChapterProgressResponse = await response.json();
      setProgress(updated);

      // Start timer
      if (!startTimeRef.current) {
        startTimeRef.current = Date.now();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    }
  }, [isAuthenticated, chapterId, progress?.status]);

  /**
   * Mark chapter as completed.
   */
  const markCompleted = useCallback(async () => {
    if (!isAuthenticated || progress?.status === 'completed') return;

    try {
      setError(null);

      const body: ChapterProgressMarkCompleted = { chapter_id: chapterId };

      const response = await fetch(`${API_BASE_URL}/api/v1/progress/complete`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        throw new Error(`Failed to mark chapter as completed: ${response.statusText}`);
      }

      const updated: ChapterProgressResponse = await response.json();
      setProgress(updated);

      // Stop timer
      if (timerRef.current) {
        clearInterval(timerRef.current);
        timerRef.current = null;
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    }
  }, [isAuthenticated, chapterId, progress?.status]);

  /**
   * Toggle bookmark status.
   */
  const toggleBookmark = useCallback(async () => {
    if (!isAuthenticated) return;

    try {
      setError(null);

      const body: ChapterProgressToggleBookmark = { chapter_id: chapterId };

      const response = await fetch(`${API_BASE_URL}/api/v1/progress/bookmark`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(body),
      });

      if (!response.ok) {
        throw new Error(`Failed to toggle bookmark: ${response.statusText}`);
      }

      const updated: ChapterProgressResponse = await response.json();
      setProgress(updated);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    }
  }, [isAuthenticated, chapterId]);

  /**
   * Handle scroll tracking.
   */
  useEffect(() => {
    const handleScroll = () => {
      const windowHeight = window.innerHeight;
      const documentHeight = document.documentElement.scrollHeight;
      const scrollTop = window.scrollY;

      // Calculate scroll depth as percentage
      const depth = (scrollTop + windowHeight) / documentHeight;
      setScrollDepth(Math.min(depth, 1));
    };

    window.addEventListener('scroll', handleScroll);
    handleScroll(); // Initial calculation

    return () => {
      window.removeEventListener('scroll', handleScroll);
    };
  }, []);

  /**
   * Handle timer tracking.
   */
  useEffect(() => {
    if (!isAuthenticated || progress?.status === 'completed') {
      return;
    }

    // Start timer if chapter is started
    if (progress?.status === 'started' && !timerRef.current) {
      startTimeRef.current = Date.now();

      timerRef.current = window.setInterval(() => {
        if (startTimeRef.current) {
          setTimeSpent(Date.now() - startTimeRef.current);
        }
      }, 1000); // Update every second
    }

    return () => {
      if (timerRef.current) {
        clearInterval(timerRef.current);
        timerRef.current = null;
      }
    };
  }, [isAuthenticated, progress?.status]);

  /**
   * Auto-mark chapter as started on mount.
   */
  useEffect(() => {
    if (autoMarkStarted && isAuthenticated && !progress) {
      markStarted();
    }
  }, [autoMarkStarted, isAuthenticated, progress, markStarted]);

  /**
   * Auto-mark as completed if thresholds are met.
   */
  useEffect(() => {
    if (
      autoMarkCompleted &&
      progress?.status === 'started' &&
      timeSpent >= autoCompleteThresholdMs &&
      scrollDepth >= SCROLL_DEPTH_THRESHOLD
    ) {
      markCompleted();
    }
  }, [
    autoMarkCompleted,
    progress?.status,
    timeSpent,
    scrollDepth,
    autoCompleteThresholdMs,
    markCompleted,
  ]);

  /**
   * Fetch progress on mount.
   */
  useEffect(() => {
    fetchProgress();
  }, [fetchProgress]);

  const shouldAutoComplete =
    timeSpent >= autoCompleteThresholdMs && scrollDepth >= SCROLL_DEPTH_THRESHOLD;

  return {
    progress,
    isLoading,
    error,
    timeSpent,
    scrollDepth,
    markStarted,
    markCompleted,
    toggleBookmark,
    isStarted: progress?.status === 'started' || progress?.status === 'completed',
    isCompleted: progress?.status === 'completed',
    isBookmarked: progress?.is_bookmarked ?? false,
    shouldAutoComplete,
  };
}

export default useChapterProgress;
