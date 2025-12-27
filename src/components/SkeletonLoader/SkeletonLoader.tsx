/**
 * SkeletonLoader Component
 *
 * Displays a shimmer loading placeholder.
 * Supports multiple variants for different content types.
 * Respects reduced motion preferences.
 *
 * @see specs/007-enhance-ui/research.md for animation strategy
 */

import React from 'react';
import clsx from 'clsx';
import styles from './SkeletonLoader.module.css';

export type SkeletonVariant = 'text' | 'title' | 'avatar' | 'thumbnail' | 'button' | 'card';

export interface SkeletonLoaderProps {
  /** Variant type determining shape and size */
  variant?: SkeletonVariant;

  /** Custom width (CSS value) */
  width?: string | number;

  /** Custom height (CSS value) */
  height?: string | number;

  /** Number of skeleton items to render */
  count?: number;

  /** Gap between multiple items (CSS value) */
  gap?: string | number;

  /** Additional CSS classes */
  className?: string;

  /** Custom border radius (CSS value) */
  borderRadius?: string | number;

  /** Inline styles */
  style?: React.CSSProperties;

  /** Accessible label for screen readers */
  'aria-label'?: string;
}

/**
 * SkeletonLoader Component
 *
 * @example
 * ```tsx
 * // Basic text placeholder
 * <SkeletonLoader variant="text" />
 *
 * // Multiple lines
 * <SkeletonLoader variant="text" count={3} gap="0.5rem" />
 *
 * // Avatar placeholder
 * <SkeletonLoader variant="avatar" />
 *
 * // Custom dimensions
 * <SkeletonLoader width={200} height={100} />
 * ```
 */
export function SkeletonLoader({
  variant = 'text',
  width,
  height,
  count = 1,
  gap = '0.5rem',
  className,
  borderRadius,
  style,
  'aria-label': ariaLabel = 'Loading...',
}: SkeletonLoaderProps): React.ReactElement {
  const customStyle: React.CSSProperties = {
    ...style,
    ...(width !== undefined && { width: typeof width === 'number' ? `${width}px` : width }),
    ...(height !== undefined && { height: typeof height === 'number' ? `${height}px` : height }),
    ...(borderRadius !== undefined && {
      '--skeleton-radius': typeof borderRadius === 'number' ? `${borderRadius}px` : borderRadius,
    } as React.CSSProperties),
  };

  const skeletonElement = (key?: number) => (
    <div
      key={key}
      className={clsx(styles.skeleton, styles[variant], className)}
      style={customStyle}
      role="status"
      aria-label={ariaLabel}
      aria-busy="true"
    />
  );

  if (count === 1) {
    return skeletonElement();
  }

  const gapValue = typeof gap === 'number' ? `${gap}px` : gap;

  return (
    <div
      style={{ display: 'flex', flexDirection: 'column', gap: gapValue }}
      role="status"
      aria-label={ariaLabel}
      aria-busy="true"
    >
      {Array.from({ length: count }, (_, i) => (
        <div
          key={i}
          className={clsx(styles.skeleton, styles[variant], className)}
          style={customStyle}
          aria-hidden="true"
        />
      ))}
    </div>
  );
}

export default SkeletonLoader;
