/**
 * CountUp Animation Component
 *
 * Animates a number counting up from 0 to target value.
 * Respects reduced motion preference.
 *
 * @see specs/007-enhance-ui/research.md for animation strategy
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useScrollReveal, type ScrollRevealOptions } from './useScrollReveal';
import { useReducedMotion } from './useReducedMotion';
import clsx from 'clsx';
import styles from './CountUp.module.css';

export interface CountUpProps extends Omit<ScrollRevealOptions, 'delay'> {
  /** Target number to count to */
  end: number;

  /** Starting number (default: 0) */
  start?: number;

  /** Animation duration in ms (default: 2000) */
  duration?: number;

  /** Optional prefix (e.g., "$") */
  prefix?: string;

  /** Optional suffix (e.g., "+", "%") */
  suffix?: string;

  /** Decimal places (default: 0) */
  decimals?: number;

  /** Whether to use separator for thousands */
  separator?: boolean;

  /** Custom separator character (default: ",") */
  separatorChar?: string;

  /** Delay before starting animation (default: 0) */
  delay?: number;

  /** Additional CSS classes */
  className?: string;

  /** HTML element type (default: 'span') */
  as?: 'span' | 'div' | 'p';
}

/**
 * Easing function for smooth animation
 */
function easeOutQuad(t: number): number {
  return t * (2 - t);
}

/**
 * Format number with separator
 */
function formatNumber(
  value: number,
  decimals: number,
  separator: boolean,
  separatorChar: string
): string {
  const fixed = value.toFixed(decimals);

  if (!separator) return fixed;

  const [intPart, decPart] = fixed.split('.');
  const formattedInt = intPart.replace(/\B(?=(\d{3})+(?!\d))/g, separatorChar);

  return decPart ? `${formattedInt}.${decPart}` : formattedInt;
}

/**
 * CountUp Animation Component
 *
 * Animates a number counting up when visible on screen.
 *
 * @example
 * ```tsx
 * // Basic usage
 * <CountUp end={1000} />
 *
 * // With formatting
 * <CountUp end={99.99} prefix="$" decimals={2} separator />
 *
 * // As percentage
 * <CountUp end={85} suffix="%" />
 * ```
 */
export function CountUp({
  end,
  start = 0,
  duration = 2000,
  prefix = '',
  suffix = '',
  decimals = 0,
  separator = false,
  separatorChar = ',',
  delay = 0,
  threshold = 0.2,
  once = true,
  rootMargin,
  className,
  as: Component = 'span',
}: CountUpProps): React.ReactElement {
  const [currentValue, setCurrentValue] = useState(start);
  const animationRef = useRef<number | null>(null);
  const startTimeRef = useRef<number | null>(null);
  const hasAnimatedRef = useRef(false);

  const { prefersReducedMotion } = useReducedMotion();
  const { ref, isVisible } = useScrollReveal({ threshold, once, rootMargin });

  // Animate the count
  const animate = useCallback(
    (timestamp: number) => {
      if (!startTimeRef.current) {
        startTimeRef.current = timestamp;
      }

      const elapsed = timestamp - startTimeRef.current;
      const progress = Math.min(elapsed / duration, 1);
      const easedProgress = easeOutQuad(progress);

      const value = start + (end - start) * easedProgress;
      setCurrentValue(value);

      if (progress < 1) {
        animationRef.current = requestAnimationFrame(animate);
      } else {
        setCurrentValue(end);
      }
    },
    [start, end, duration]
  );

  // Start animation when visible
  useEffect(() => {
    // If reduced motion, show final value immediately
    if (prefersReducedMotion) {
      setCurrentValue(end);
      return;
    }

    // Start animation when visible and not already animated
    if (isVisible && !hasAnimatedRef.current) {
      hasAnimatedRef.current = true;

      const timeoutId = setTimeout(() => {
        startTimeRef.current = null;
        animationRef.current = requestAnimationFrame(animate);
      }, delay);

      return () => {
        clearTimeout(timeoutId);
        if (animationRef.current) {
          cancelAnimationFrame(animationRef.current);
        }
      };
    }

    return undefined;
  }, [isVisible, prefersReducedMotion, animate, delay, end]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (animationRef.current) {
        cancelAnimationFrame(animationRef.current);
      }
    };
  }, []);

  const displayValue = formatNumber(
    currentValue,
    decimals,
    separator,
    separatorChar
  );

  return (
    <Component
      ref={ref as React.RefObject<HTMLElement | null>}
      className={clsx(styles.countUp, className)}
      aria-label={`${prefix}${formatNumber(end, decimals, separator, separatorChar)}${suffix}`}
    >
      {prefix}
      <span className={styles.value}>{displayValue}</span>
      {suffix}
    </Component>
  );
}

export default CountUp;
