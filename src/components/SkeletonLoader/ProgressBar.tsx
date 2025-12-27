/**
 * ProgressBar Component
 *
 * Linear progress indicator with determinate and indeterminate modes.
 * Respects reduced motion preferences.
 *
 * @see specs/007-enhance-ui/research.md for animation strategy
 */

import React from 'react';
import clsx from 'clsx';
import styles from './ProgressBar.module.css';

export type ProgressBarSize = 'small' | 'medium' | 'large';
export type ProgressBarColor = 'primary' | 'success' | 'warning' | 'error';

export interface ProgressBarProps {
  /** Current progress value (0-100). If undefined, shows indeterminate animation. */
  value?: number;

  /** Maximum value (default: 100) */
  max?: number;

  /** Size variant */
  size?: ProgressBarSize;

  /** Color variant */
  color?: ProgressBarColor;

  /** Show percentage label */
  showLabel?: boolean;

  /** Custom label text */
  label?: string;

  /** Additional CSS classes */
  className?: string;

  /** Inline styles */
  style?: React.CSSProperties;

  /** Accessible label for screen readers */
  'aria-label'?: string;
}

/**
 * ProgressBar Component
 *
 * @example
 * ```tsx
 * // Determinate progress
 * <ProgressBar value={75} />
 *
 * // Indeterminate (loading)
 * <ProgressBar />
 *
 * // With label
 * <ProgressBar value={50} showLabel label="Uploading..." />
 *
 * // Success color
 * <ProgressBar value={100} color="success" />
 * ```
 */
export function ProgressBar({
  value,
  max = 100,
  size = 'medium',
  color = 'primary',
  showLabel = false,
  label,
  className,
  style,
  'aria-label': ariaLabel,
}: ProgressBarProps): React.ReactElement {
  const isIndeterminate = value === undefined;
  const percentage = isIndeterminate ? 0 : Math.min(100, Math.max(0, (value / max) * 100));
  const displayPercentage = Math.round(percentage);

  const accessibleLabel = ariaLabel || label || (isIndeterminate ? 'Loading...' : `${displayPercentage}% complete`);

  return (
    <div
      className={clsx(
        styles.progressContainer,
        {
          [styles.small]: size === 'small',
          [styles.large]: size === 'large',
          [styles.indeterminate]: isIndeterminate,
          [styles.success]: color === 'success',
          [styles.warning]: color === 'warning',
          [styles.error]: color === 'error',
        },
        className
      )}
      style={style}
    >
      {showLabel && (label || !isIndeterminate) && (
        <div className={styles.progressLabel}>
          {label && <span className={styles.progressLabelText}>{label}</span>}
          {!isIndeterminate && (
            <span className={styles.progressLabelValue}>{displayPercentage}%</span>
          )}
        </div>
      )}
      <div
        className={styles.progressTrack}
        role="progressbar"
        aria-valuenow={isIndeterminate ? undefined : percentage}
        aria-valuemin={0}
        aria-valuemax={max}
        aria-label={accessibleLabel}
      >
        <div
          className={styles.progressFill}
          style={isIndeterminate ? undefined : { width: `${percentage}%` }}
        />
      </div>
      <span className={styles.srOnly}>{accessibleLabel}</span>
    </div>
  );
}

export default ProgressBar;
