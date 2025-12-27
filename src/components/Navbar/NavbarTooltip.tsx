/**
 * NavbarTooltip Component
 *
 * Accessible tooltip for navbar items.
 * Appears on hover/focus with proper ARIA attributes.
 *
 * @see specs/007-enhance-ui/plan.md for design decisions
 */

import React, { useState, useRef, useCallback, useId } from 'react';
import clsx from 'clsx';
import styles from './NavbarTooltip.module.css';

export type TooltipPosition = 'top' | 'bottom' | 'left' | 'right';

export interface NavbarTooltipProps {
  /** Tooltip content */
  content: string;

  /** Position relative to trigger element */
  position?: TooltipPosition;

  /** Delay before showing tooltip (ms) */
  showDelay?: number;

  /** Delay before hiding tooltip (ms) */
  hideDelay?: number;

  /** Children that trigger the tooltip */
  children: React.ReactElement;

  /** Additional CSS classes for tooltip */
  className?: string;

  /** Whether tooltip is disabled */
  disabled?: boolean;
}

/**
 * NavbarTooltip Component
 *
 * @example
 * ```tsx
 * <NavbarTooltip content="Search documentation">
 *   <button>
 *     <SearchIcon />
 *   </button>
 * </NavbarTooltip>
 * ```
 */
export function NavbarTooltip({
  content,
  position = 'bottom',
  showDelay = 200,
  hideDelay = 0,
  children,
  className,
  disabled = false,
}: NavbarTooltipProps): React.ReactElement {
  const [isVisible, setIsVisible] = useState(false);
  const showTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const hideTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const tooltipId = useId();

  const clearTimeouts = useCallback(() => {
    if (showTimeoutRef.current) {
      clearTimeout(showTimeoutRef.current);
      showTimeoutRef.current = null;
    }
    if (hideTimeoutRef.current) {
      clearTimeout(hideTimeoutRef.current);
      hideTimeoutRef.current = null;
    }
  }, []);

  const showTooltip = useCallback(() => {
    if (disabled) return;
    clearTimeouts();
    showTimeoutRef.current = setTimeout(() => {
      setIsVisible(true);
    }, showDelay);
  }, [disabled, showDelay, clearTimeouts]);

  const hideTooltip = useCallback(() => {
    clearTimeouts();
    hideTimeoutRef.current = setTimeout(() => {
      setIsVisible(false);
    }, hideDelay);
  }, [hideDelay, clearTimeouts]);

  // Clone child to add event handlers and aria attributes
  const trigger = React.cloneElement(children, {
    onMouseEnter: (e: React.MouseEvent) => {
      showTooltip();
      children.props.onMouseEnter?.(e);
    },
    onMouseLeave: (e: React.MouseEvent) => {
      hideTooltip();
      children.props.onMouseLeave?.(e);
    },
    onFocus: (e: React.FocusEvent) => {
      showTooltip();
      children.props.onFocus?.(e);
    },
    onBlur: (e: React.FocusEvent) => {
      hideTooltip();
      children.props.onBlur?.(e);
    },
    'aria-describedby': isVisible ? tooltipId : undefined,
  });

  return (
    <div className={styles.wrapper}>
      {trigger}
      {isVisible && !disabled && (
        <div
          id={tooltipId}
          role="tooltip"
          className={clsx(
            styles.tooltip,
            styles[position],
            className
          )}
        >
          {content}
          <span className={styles.arrow} aria-hidden="true" />
        </div>
      )}
    </div>
  );
}

export default NavbarTooltip;
