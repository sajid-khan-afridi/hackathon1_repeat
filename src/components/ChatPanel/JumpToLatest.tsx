/**
 * JumpToLatest Component
 *
 * Button that appears when user scrolls up, allowing quick jump to latest messages.
 *
 * @see specs/007-enhance-ui/data-model.md for JumpToLatestProps
 */

import React from 'react';
import { ChevronDown } from 'lucide-react';
import clsx from 'clsx';
import styles from './JumpToLatest.module.css';

export interface JumpToLatestProps {
  /** Whether button is visible */
  visible: boolean;

  /** Callback when clicked */
  onClick: () => void;

  /** Number of messages below current view */
  newMessageCount?: number;

  /** Additional CSS classes */
  className?: string;
}

/**
 * JumpToLatest Component
 *
 * Shows a floating button to scroll to the most recent messages.
 */
export function JumpToLatest({
  visible,
  onClick,
  newMessageCount = 0,
  className,
}: JumpToLatestProps): React.ReactElement | null {
  if (!visible) {
    return null;
  }

  return (
    <button
      type="button"
      className={clsx(styles.button, className)}
      onClick={onClick}
      aria-label={
        newMessageCount > 0
          ? `Jump to latest - ${newMessageCount} new message${newMessageCount === 1 ? '' : 's'}`
          : 'Jump to latest'
      }
    >
      <ChevronDown size={20} aria-hidden="true" />
      {newMessageCount > 0 && <span className={styles.count}>{newMessageCount}</span>}
    </button>
  );
}

export default JumpToLatest;
