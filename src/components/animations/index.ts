/**
 * Animation Components and Hooks Index
 *
 * Exports all animation utilities for Phase 6 UI enhancements.
 *
 * @see specs/007-enhance-ui/plan.md for component structure
 */

// Hooks
export { useReducedMotion, type UseReducedMotionReturn } from './useReducedMotion';
export { useScrollReveal, type UseScrollRevealReturn, type ScrollRevealOptions } from './useScrollReveal';
export { useAnimationState, type UseAnimationStateReturn, type AnimationConfig, type AnimationPhase } from './useAnimationState';

// Components
export { FadeIn, type FadeInProps, type FadeInVariant } from './FadeIn';
export { SlideIn, type SlideInProps, type SlideInDirection } from './SlideIn';
export { CountUp, type CountUpProps } from './CountUp';
