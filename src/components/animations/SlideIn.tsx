/**
 * SlideIn Animation Component
 *
 * A wrapper component that applies slide-in animation to its children.
 * Ideal for panels, modals, and other UI elements that slide in/out.
 *
 * @see specs/007-enhance-ui/research.md for animation strategy
 */

import React, { ReactNode, ElementType } from 'react';
import clsx from 'clsx';
import { useAnimationState } from './useAnimationState';
import { useReducedMotion } from './useReducedMotion';
import styles from './SlideIn.module.css';

/**
 * SlideIn direction variants
 */
export type SlideInDirection = 'left' | 'right' | 'top' | 'bottom';

/**
 * SlideIn component props
 */
export interface SlideInProps {
  /** Content to animate */
  children: ReactNode;

  /** Whether the element is visible */
  isVisible: boolean;

  /** Direction to slide from (default: 'right') */
  direction?: SlideInDirection;

  /** Enter animation duration in ms (default: 300) */
  enterDuration?: number;

  /** Exit animation duration in ms (default: 200) */
  exitDuration?: number;

  /** Callback when enter animation completes */
  onEnterComplete?: () => void;

  /** Callback when exit animation completes */
  onExitComplete?: () => void;

  /** Additional CSS class names */
  className?: string;

  /** HTML element type (default: 'div') */
  as?: ElementType;

  /** Additional styles */
  style?: React.CSSProperties;

  /** Enable RTL awareness (reverses left/right directions in RTL) */
  rtlAware?: boolean;
}

/**
 * SlideIn Animation Component
 *
 * Provides slide-in/out animations for panels and modals.
 * Automatically handles enter/exit states and cleanup.
 *
 * @example
 * ```tsx
 * // Basic slide-in from right
 * <SlideIn isVisible={isPanelOpen} direction="right">
 *   <Panel>Content</Panel>
 * </SlideIn>
 *
 * // With callbacks
 * <SlideIn
 *   isVisible={isOpen}
 *   direction="bottom"
 *   onExitComplete={() => setIsRemoved(true)}
 * >
 *   <Modal>Content</Modal>
 * </SlideIn>
 *
 * // RTL-aware (flips left/right in RTL mode)
 * <SlideIn isVisible={isOpen} direction="right" rtlAware>
 *   <Sidebar />
 * </SlideIn>
 * ```
 */
export function SlideIn({
  children,
  isVisible,
  direction = 'right',
  enterDuration = 300,
  exitDuration = 200,
  onEnterComplete,
  onExitComplete,
  className,
  as: Component = 'div',
  style: additionalStyle,
  rtlAware = false,
}: SlideInProps): React.ReactElement | null {
  const { prefersReducedMotion } = useReducedMotion();

  // Detect RTL mode
  const [isRTL, setIsRTL] = React.useState(false);

  React.useEffect(() => {
    if (typeof document !== 'undefined') {
      setIsRTL(document.documentElement.dir === 'rtl');
    }
  }, []);

  // Adjust direction for RTL if enabled
  const effectiveDirection = React.useMemo(() => {
    if (!rtlAware || !isRTL) {
      return direction;
    }

    // Flip horizontal directions in RTL
    if (direction === 'left') return 'right';
    if (direction === 'right') return 'left';
    return direction;
  }, [direction, rtlAware, isRTL]);

  // Use animation state hook
  const { phase, shouldRender, style: animationStyle } = useAnimationState({
    isVisible,
    enterDuration,
    exitDuration,
    onEnterComplete,
    onExitComplete,
  });

  // Don't render if not needed
  if (!shouldRender) {
    return null;
  }

  // Build class names
  const classNames = clsx(
    styles.slideIn,
    styles[`direction_${effectiveDirection}`],
    styles[`phase_${phase}`],
    {
      [styles.reducedMotion]: prefersReducedMotion,
    },
    className
  );

  // Build style object
  const style: React.CSSProperties = {
    ...animationStyle,
    ...additionalStyle,
  };

  return (
    <Component className={classNames} style={style}>
      {children}
    </Component>
  );
}

export default SlideIn;
