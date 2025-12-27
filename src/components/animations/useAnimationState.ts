/**
 * useAnimationState Hook
 *
 * Manages animation state transitions for components that need to track
 * entering, entered, exiting, and exited animation phases.
 *
 * @see specs/007-enhance-ui/data-model.md for state machine definitions
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { useReducedMotion } from './useReducedMotion';

/**
 * Animation phases
 */
export type AnimationPhase = 'initial' | 'entering' | 'entered' | 'exiting' | 'exited';

/**
 * Animation configuration
 */
export interface AnimationConfig {
  /** Duration for enter animation in ms (default: 300) */
  enterDuration?: number;

  /** Duration for exit animation in ms (default: 200) */
  exitDuration?: number;

  /** Whether the element is visible/mounted */
  isVisible: boolean;

  /** Callback when enter animation completes */
  onEnterComplete?: () => void;

  /** Callback when exit animation completes */
  onExitComplete?: () => void;
}

/**
 * Return type for useAnimationState hook
 */
export interface UseAnimationStateReturn {
  /** Current animation phase */
  phase: AnimationPhase;

  /** Whether the element should be rendered */
  shouldRender: boolean;

  /** CSS class names for current phase */
  className: string;

  /** Get style object with animation duration */
  style: React.CSSProperties;

  /** Force immediate visibility (skip animation) */
  skipAnimation: () => void;
}

/**
 * Custom hook for managing component animation state transitions.
 *
 * Provides a state machine for enter/exit animations with proper timing.
 * Respects the user's prefers-reduced-motion preference.
 *
 * @param config - Animation configuration
 *
 * @example
 * ```tsx
 * function Modal({ isOpen, onClose }) {
 *   const { phase, shouldRender, className, style } = useAnimationState({
 *     isVisible: isOpen,
 *     enterDuration: 300,
 *     exitDuration: 200,
 *     onExitComplete: onClose,
 *   });
 *
 *   if (!shouldRender) return null;
 *
 *   return (
 *     <div className={clsx(styles.modal, className)} style={style}>
 *       Modal content
 *     </div>
 *   );
 * }
 * ```
 */
export function useAnimationState(config: AnimationConfig): UseAnimationStateReturn {
  const {
    isVisible,
    enterDuration = 300,
    exitDuration = 200,
    onEnterComplete,
    onExitComplete,
  } = config;

  const [phase, setPhase] = useState<AnimationPhase>(() =>
    isVisible ? 'entered' : 'initial'
  );
  const [shouldRender, setShouldRender] = useState(isVisible);

  const { prefersReducedMotion, getAdjustedDuration } = useReducedMotion();

  const timeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const prevVisibleRef = useRef(isVisible);

  // Cleanup timeout on unmount
  useEffect(() => {
    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, []);

  // Handle visibility changes
  useEffect(() => {
    const prevVisible = prevVisibleRef.current;
    prevVisibleRef.current = isVisible;

    // Clear any existing timeout
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
      timeoutRef.current = null;
    }

    if (isVisible && !prevVisible) {
      // Entering
      setShouldRender(true);

      if (prefersReducedMotion) {
        // Skip animation for reduced motion
        setPhase('entered');
        onEnterComplete?.();
      } else {
        setPhase('entering');

        timeoutRef.current = setTimeout(() => {
          setPhase('entered');
          onEnterComplete?.();
        }, getAdjustedDuration(enterDuration));
      }
    } else if (!isVisible && prevVisible) {
      // Exiting
      if (prefersReducedMotion) {
        // Skip animation for reduced motion
        setPhase('exited');
        setShouldRender(false);
        onExitComplete?.();
      } else {
        setPhase('exiting');

        timeoutRef.current = setTimeout(() => {
          setPhase('exited');
          setShouldRender(false);
          onExitComplete?.();
        }, getAdjustedDuration(exitDuration));
      }
    }
  }, [
    isVisible,
    enterDuration,
    exitDuration,
    prefersReducedMotion,
    getAdjustedDuration,
    onEnterComplete,
    onExitComplete,
  ]);

  // Generate class name based on phase
  const getClassName = useCallback((): string => {
    const baseClass = 'animation-state';
    const phaseClass = `animation-state--${phase}`;

    return `${baseClass} ${phaseClass}`;
  }, [phase]);

  // Generate style object with duration
  const getStyle = useCallback((): React.CSSProperties => {
    const duration =
      phase === 'entering'
        ? getAdjustedDuration(enterDuration)
        : phase === 'exiting'
          ? getAdjustedDuration(exitDuration)
          : 0;

    return {
      transitionDuration: `${duration}ms`,
      animationDuration: `${duration}ms`,
    };
  }, [phase, enterDuration, exitDuration, getAdjustedDuration]);

  // Force skip animation
  const skipAnimation = useCallback(() => {
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
      timeoutRef.current = null;
    }

    if (isVisible) {
      setPhase('entered');
      onEnterComplete?.();
    } else {
      setPhase('exited');
      setShouldRender(false);
      onExitComplete?.();
    }
  }, [isVisible, onEnterComplete, onExitComplete]);

  return {
    phase,
    shouldRender,
    className: getClassName(),
    style: getStyle(),
    skipAnimation,
  };
}

export default useAnimationState;
