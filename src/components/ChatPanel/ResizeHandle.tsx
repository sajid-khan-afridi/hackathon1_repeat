/**
 * ResizeHandle Component
 *
 * Draggable handle for resizing the chat panel width.
 *
 * @see specs/007-enhance-ui/data-model.md for ResizeHandleProps
 */

import React, { useRef, useCallback, useEffect, useState } from 'react';
import clsx from 'clsx';
import styles from './ResizeHandle.module.css';

export interface ResizeHandleProps {
  /** Callback during resize */
  onResize: (width: number) => void;

  /** Callback when resize starts */
  onResizeStart?: () => void;

  /** Callback when resize ends */
  onResizeEnd?: () => void;

  /** Current panel width */
  currentWidth: number;

  /** Minimum allowed width */
  minWidth: number;

  /** Maximum allowed width */
  maxWidth: number;

  /** RTL mode (handle on right edge instead of left) */
  isRTL?: boolean;

  /** Additional CSS classes */
  className?: string;
}

/**
 * ResizeHandle Component
 *
 * Provides a draggable handle for resizing the chat panel.
 * Supports both mouse and keyboard interactions for accessibility.
 */
export function ResizeHandle({
  onResize,
  onResizeStart,
  onResizeEnd,
  currentWidth,
  minWidth,
  maxWidth,
  isRTL = false,
  className,
}: ResizeHandleProps): React.ReactElement {
  const [isResizing, setIsResizing] = useState(false);
  const startXRef = useRef(0);
  const startWidthRef = useRef(currentWidth);

  // Handle mouse move during resize
  const handleMouseMove = useCallback(
    (e: MouseEvent) => {
      if (!isResizing) return;

      const deltaX = isRTL
        ? e.clientX - startXRef.current
        : startXRef.current - e.clientX;

      const newWidth = Math.min(maxWidth, Math.max(minWidth, startWidthRef.current + deltaX));
      onResize(newWidth);
    },
    [isResizing, isRTL, maxWidth, minWidth, onResize]
  );

  // Handle mouse up (end resize)
  const handleMouseUp = useCallback(() => {
    if (isResizing) {
      setIsResizing(false);
      onResizeEnd?.();
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    }
  }, [isResizing, onResizeEnd]);

  // Add/remove global event listeners
  useEffect(() => {
    if (isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      document.body.style.cursor = 'col-resize';
      document.body.style.userSelect = 'none';

      return () => {
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
      };
    }
    return undefined;
  }, [isResizing, handleMouseMove, handleMouseUp]);

  // Handle mouse down (start resize)
  const handleMouseDown = useCallback(
    (e: React.MouseEvent) => {
      e.preventDefault();
      startXRef.current = e.clientX;
      startWidthRef.current = currentWidth;
      setIsResizing(true);
      onResizeStart?.();
    },
    [currentWidth, onResizeStart]
  );

  // Handle keyboard resize
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      const step = e.shiftKey ? 50 : 10;
      let newWidth = currentWidth;

      switch (e.key) {
        case 'ArrowLeft':
          newWidth = isRTL ? currentWidth + step : currentWidth - step;
          break;
        case 'ArrowRight':
          newWidth = isRTL ? currentWidth - step : currentWidth + step;
          break;
        case 'Home':
          newWidth = minWidth;
          break;
        case 'End':
          newWidth = maxWidth;
          break;
        default:
          return;
      }

      e.preventDefault();
      onResize(Math.min(maxWidth, Math.max(minWidth, newWidth)));
    },
    [currentWidth, isRTL, maxWidth, minWidth, onResize]
  );

  return (
    <div
      className={clsx(
        styles.handle,
        { [styles.resizing]: isResizing, [styles.rtl]: isRTL },
        className
      )}
      onMouseDown={handleMouseDown}
      onKeyDown={handleKeyDown}
      role="separator"
      aria-orientation="vertical"
      aria-label="Resize chat panel"
      aria-valuenow={currentWidth}
      aria-valuemin={minWidth}
      aria-valuemax={maxWidth}
      tabIndex={0}
    >
      <div className={styles.handleIndicator} aria-hidden="true" />
    </div>
  );
}

export default ResizeHandle;
