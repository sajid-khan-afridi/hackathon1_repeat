/**
 * FadeIn Animation Component
 *
 * A wrapper component that applies fade-in animation to its children.
 * Supports multiple animation variants and respects reduced motion preference.
 *
 * @see specs/007-enhance-ui/research.md for animation strategy
 */

import React, { ReactNode, ElementType } from 'react';
import clsx from 'clsx';
import { useScrollReveal, type ScrollRevealOptions } from './useScrollReveal';
import { useReducedMotion } from './useReducedMotion';
import styles from './FadeIn.module.css';

/**
 * FadeIn animation variants
 */
export type FadeInVariant = 'fade' | 'fade-up' | 'fade-down' | 'fade-left' | 'fade-right';

/**
 * FadeIn component props
 */
export interface FadeInProps extends ScrollRevealOptions {
  /** Content to animate */
  children: ReactNode;

  /** Animation variant (default: 'fade-up') */
  variant?: FadeInVariant;

  /** Animation duration in ms (default: 400) */
  duration?: number;

  /** Stagger index for staggered animations */
  staggerIndex?: number;

  /** Base stagger delay in ms (default: 50) */
  staggerDelay?: number;

  /** Whether to trigger on scroll (default: true) */
  triggerOnScroll?: boolean;

  /** Additional CSS class names */
  className?: string;

  /** HTML element type (default: 'div') */
  as?: ElementType;

  /** Additional HTML attributes */
  style?: React.CSSProperties;
}

/**
 * FadeIn Animation Component
 *
 * Wraps content with a fade animation that can be triggered on scroll
 * or immediately on mount. Supports multiple animation directions.
 *
 * @example
 * ```tsx
 * // Basic fade-up animation triggered on scroll
 * <FadeIn>
 *   <h1>Hello World</h1>
 * </FadeIn>
 *
 * // Staggered children
 * <div>
 *   {items.map((item, i) => (
 *     <FadeIn key={item.id} staggerIndex={i}>
 *       {item.content}
 *     </FadeIn>
 *   ))}
 * </div>
 *
 * // Immediate animation (no scroll trigger)
 * <FadeIn triggerOnScroll={false} variant="fade-left">
 *   <Card>Content</Card>
 * </FadeIn>
 * ```
 */
export function FadeIn({
  children,
  variant = 'fade-up',
  duration = 400,
  staggerIndex = 0,
  staggerDelay = 50,
  triggerOnScroll = true,
  threshold = 0.2,
  once = true,
  rootMargin,
  delay: baseDelay = 0,
  className,
  as: Component = 'div',
  style: additionalStyle,
}: FadeInProps): React.ReactElement {
  const { prefersReducedMotion } = useReducedMotion();

  // Calculate total delay including stagger
  const totalDelay = baseDelay + staggerIndex * staggerDelay;

  // Use scroll reveal if triggered on scroll
  const { ref, isVisible } = useScrollReveal({
    threshold,
    once,
    rootMargin,
    delay: triggerOnScroll ? totalDelay : 0,
  });

  // Determine visibility: if not scroll-triggered, show immediately after delay
  const [immediateVisible, setImmediateVisible] = React.useState(!triggerOnScroll);

  React.useEffect(() => {
    if (!triggerOnScroll && !immediateVisible) {
      const timer = setTimeout(() => {
        setImmediateVisible(true);
      }, totalDelay);

      return () => clearTimeout(timer);
    }
    return undefined;
  }, [triggerOnScroll, immediateVisible, totalDelay]);

  const visible = triggerOnScroll ? isVisible : immediateVisible;

  // Calculate actual duration based on reduced motion preference
  const actualDuration = prefersReducedMotion ? 0.01 : duration;

  // Build style object
  const style: React.CSSProperties = {
    '--fade-duration': `${actualDuration}ms`,
    '--fade-delay': triggerOnScroll ? '0ms' : `${totalDelay}ms`,
    ...additionalStyle,
  } as React.CSSProperties;

  // Build class names
  const classNames = clsx(
    styles.fadeIn,
    styles[variant.replace('-', '_')],
    {
      [styles.visible]: visible,
      [styles.reducedMotion]: prefersReducedMotion,
    },
    className
  );

  return (
    <Component
      ref={triggerOnScroll ? (ref as React.RefObject<HTMLDivElement>) : undefined}
      className={classNames}
      style={style}
    >
      {children}
    </Component>
  );
}

export default FadeIn;
