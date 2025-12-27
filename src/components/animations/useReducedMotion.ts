/**
 * useReducedMotion Hook
 *
 * Detects and respects the user's prefers-reduced-motion system preference.
 * This hook follows WCAG 2.2 AA guidelines for animation accessibility.
 *
 * @see specs/007-enhance-ui/data-model.md for interface definitions
 * @see specs/007-enhance-ui/research.md for reduced motion strategy
 */

import { useState, useEffect, useCallback } from 'react';

/**
 * Return type for useReducedMotion hook
 */
export interface UseReducedMotionReturn {
  /** Whether user prefers reduced motion */
  prefersReducedMotion: boolean;

  /** Get animation duration adjusted for preference */
  getAdjustedDuration: (baseDuration: number) => number;

  /** Whether to skip animation entirely */
  shouldSkipAnimation: boolean;
}

/**
 * Custom hook that detects the user's motion preference
 * and provides utilities for adjusting animations accordingly.
 *
 * @example
 * ```tsx
 * const { prefersReducedMotion, getAdjustedDuration } = useReducedMotion();
 *
 * const duration = getAdjustedDuration(300); // Returns 0.01 if reduced motion
 *
 * return (
 *   <div style={{ transitionDuration: `${duration}ms` }}>
 *     Content
 *   </div>
 * );
 * ```
 */
export function useReducedMotion(): UseReducedMotionReturn {
  const [prefersReducedMotion, setPrefersReducedMotion] = useState<boolean>(() => {
    // Check for SSR (server-side rendering)
    if (typeof window === 'undefined') {
      return false;
    }
    return window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  });

  useEffect(() => {
    // Check for SSR
    if (typeof window === 'undefined') {
      return;
    }

    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');

    // Set initial value
    setPrefersReducedMotion(mediaQuery.matches);

    // Handler for changes
    const handleChange = (event: MediaQueryListEvent) => {
      setPrefersReducedMotion(event.matches);
    };

    // Add event listener
    mediaQuery.addEventListener('change', handleChange);

    // Cleanup
    return () => {
      mediaQuery.removeEventListener('change', handleChange);
    };
  }, []);

  /**
   * Returns adjusted duration based on motion preference
   * When reduced motion is preferred, returns near-instant duration (0.01ms)
   */
  const getAdjustedDuration = useCallback(
    (baseDuration: number): number => {
      return prefersReducedMotion ? 0.01 : baseDuration;
    },
    [prefersReducedMotion]
  );

  return {
    prefersReducedMotion,
    getAdjustedDuration,
    shouldSkipAnimation: prefersReducedMotion,
  };
}

export default useReducedMotion;
