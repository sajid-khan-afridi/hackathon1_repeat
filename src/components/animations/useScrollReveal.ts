/**
 * useScrollReveal Hook
 *
 * Provides scroll-triggered animations using IntersectionObserver.
 * Respects prefers-reduced-motion system preference.
 *
 * @see specs/007-enhance-ui/research.md for scroll animation strategy
 * @see specs/007-enhance-ui/data-model.md for interface definitions
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import { useReducedMotion } from './useReducedMotion';

/**
 * Options for scroll reveal animation
 */
export interface ScrollRevealOptions {
  /** Visibility threshold to trigger (0-1, default 0.2) */
  threshold?: number;

  /** Whether to animate only once (default true) */
  once?: boolean;

  /** Root margin for IntersectionObserver */
  rootMargin?: string;

  /** Animation delay in ms */
  delay?: number;
}

/**
 * Return type for useScrollReveal hook
 */
export interface UseScrollRevealReturn {
  /** Ref to attach to element */
  ref: React.RefObject<HTMLElement | null>;

  /** Whether element is visible */
  isVisible: boolean;

  /** CSS class names for animation state */
  className: string;

  /** Whether animation has triggered (for once-only animations) */
  hasTriggered: boolean;
}

/**
 * Custom hook for scroll-triggered reveal animations.
 *
 * Uses IntersectionObserver for better performance compared to scroll listeners.
 * Respects the user's prefers-reduced-motion preference.
 *
 * @param options - Configuration options
 *
 * @example
 * ```tsx
 * function Section() {
 *   const { ref, isVisible, className } = useScrollReveal({ threshold: 0.2 });
 *
 *   return (
 *     <section
 *       ref={ref}
 *       className={clsx(styles.section, className)}
 *     >
 *       {content}
 *     </section>
 *   );
 * }
 * ```
 */
export function useScrollReveal(options: ScrollRevealOptions = {}): UseScrollRevealReturn {
  const { threshold = 0.2, once = true, rootMargin = '0px', delay = 0 } = options;

  const ref = useRef<HTMLElement>(null);
  const [isVisible, setIsVisible] = useState(false);
  const [hasTriggered, setHasTriggered] = useState(false);
  const { prefersReducedMotion } = useReducedMotion();

  useEffect(() => {
    const element = ref.current;

    // SSR check
    if (typeof window === 'undefined' || !element) {
      return;
    }

    // If reduced motion is preferred, show element immediately
    if (prefersReducedMotion) {
      setIsVisible(true);
      setHasTriggered(true);
      return;
    }

    // If already triggered and once is true, don't observe again
    if (once && hasTriggered) {
      return;
    }

    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            // Apply delay if specified
            if (delay > 0) {
              const timeoutId = setTimeout(() => {
                setIsVisible(true);
                setHasTriggered(true);
              }, delay);

              // Store timeout ID for cleanup
              element.dataset.scrollRevealTimeout = String(timeoutId);
            } else {
              setIsVisible(true);
              setHasTriggered(true);
            }

            // Disconnect if animating only once
            if (once) {
              observer.disconnect();
            }
          } else if (!once) {
            setIsVisible(false);
          }
        });
      },
      {
        threshold,
        rootMargin,
      }
    );

    observer.observe(element);

    return () => {
      observer.disconnect();

      // Clear any pending timeout
      const timeoutId = element.dataset.scrollRevealTimeout;
      if (timeoutId) {
        clearTimeout(Number(timeoutId));
        delete element.dataset.scrollRevealTimeout;
      }
    };
  }, [threshold, once, rootMargin, delay, prefersReducedMotion, hasTriggered]);

  // Generate class name based on visibility state
  const getClassName = useCallback(() => {
    if (prefersReducedMotion) {
      return 'scroll-reveal scroll-reveal--visible';
    }

    const baseClass = 'scroll-reveal';
    const visibilityClass = isVisible ? 'scroll-reveal--visible' : 'scroll-reveal--hidden';

    return `${baseClass} ${visibilityClass}`;
  }, [isVisible, prefersReducedMotion]);

  return {
    ref,
    isVisible,
    className: getClassName(),
    hasTriggered,
  };
}

export default useScrollReveal;
