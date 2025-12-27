/**
 * UnreadBadge Component
 *
 * Displays count of unread messages with optional pulse animation.
 *
 * @see specs/007-enhance-ui/data-model.md for UnreadBadgeProps
 */

import React from 'react';
import clsx from 'clsx';
import styles from './UnreadBadge.module.css';

export interface UnreadBadgeProps {
  /** Number of unread messages */
  count: number;

  /** Maximum count to display (shows "99+" for higher) */
  maxCount?: number;

  /** Whether to show pulse animation */
  animate?: boolean;

  /** Size variant */
  size?: 'sm' | 'md' | 'lg';

  /** Additional CSS classes */
  className?: string;
}

/**
 * UnreadBadge Component
 *
 * Shows the number of unread messages with a visual badge.
 * Pulses when new messages arrive.
 */
export function UnreadBadge({
  count,
  maxCount = 99,
  animate = true,
  size = 'md',
  className,
}: UnreadBadgeProps): React.ReactElement | null {
  if (count <= 0) {
    return null;
  }

  const displayCount = count > maxCount ? `${maxCount}+` : String(count);

  return (
    <span
      className={clsx(
        styles.badge,
        styles[size],
        { [styles.animate]: animate && count > 0 },
        className
      )}
      aria-label={`${count} unread ${count === 1 ? 'message' : 'messages'}`}
    >
      {displayCount}
    </span>
  );
}

export default UnreadBadge;
