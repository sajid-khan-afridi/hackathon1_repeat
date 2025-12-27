/**
 * TypingIndicator Component
 *
 * Displays animated bouncing dots to indicate typing state.
 * Commonly used in chat interfaces.
 * Respects reduced motion preferences.
 *
 * @see specs/007-enhance-ui/research.md for animation strategy
 */

import React from 'react';
import clsx from 'clsx';
import styles from './TypingIndicator.module.css';

export type TypingIndicatorSize = 'small' | 'medium' | 'large';

export interface TypingIndicatorProps {
  /** Size variant */
  size?: TypingIndicatorSize;

  /** Additional CSS classes */
  className?: string;

  /** Accessible label for screen readers */
  label?: string;

  /** Inline styles */
  style?: React.CSSProperties;
}

/**
 * TypingIndicator Component
 *
 * @example
 * ```tsx
 * // Default size
 * <TypingIndicator />
 *
 * // Small size
 * <TypingIndicator size="small" />
 *
 * // Custom label
 * <TypingIndicator label="Assistant is typing..." />
 * ```
 */
export function TypingIndicator({
  size = 'medium',
  className,
  label = 'Typing...',
  style,
}: TypingIndicatorProps): React.ReactElement {
  return (
    <div
      className={clsx(
        styles.typingIndicator,
        {
          [styles.small]: size === 'small',
          [styles.large]: size === 'large',
        },
        className
      )}
      style={style}
      role="status"
      aria-live="polite"
    >
      <span className={styles.dot} aria-hidden="true" />
      <span className={styles.dot} aria-hidden="true" />
      <span className={styles.dot} aria-hidden="true" />
      <span className={styles.srOnly}>{label}</span>
    </div>
  );
}

export default TypingIndicator;
