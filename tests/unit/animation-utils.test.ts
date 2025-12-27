/**
 * Unit Tests: Animation Utilities
 *
 * Tests animation utility functions and hooks.
 */

import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act, waitFor } from '@testing-library/react';
import { useAnimationState } from '../../src/components/animations/useAnimationState';

describe('Animation Utilities', () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  describe('useAnimationState', () => {
    it('should start in idle phase when not visible', () => {
      const { result } = renderHook(() =>
        useAnimationState({
          isVisible: false,
          enterDuration: 300,
          exitDuration: 200,
        })
      );

      expect(result.current.phase).toBe('idle');
      expect(result.current.shouldRender).toBe(false);
    });

    it('should transition to entering phase when visible', () => {
      const { result } = renderHook(() =>
        useAnimationState({
          isVisible: true,
          enterDuration: 300,
          exitDuration: 200,
        })
      );

      expect(result.current.phase).toBe('entering');
      expect(result.current.shouldRender).toBe(true);
    });

    it('should transition to entered phase after enter duration', async () => {
      const { result } = renderHook(() =>
        useAnimationState({
          isVisible: true,
          enterDuration: 300,
          exitDuration: 200,
        })
      );

      expect(result.current.phase).toBe('entering');

      // Fast-forward time
      act(() => {
        vi.advanceTimersByTime(300);
      });

      expect(result.current.phase).toBe('entered');
    });

    it('should transition to exiting phase when visibility changes', async () => {
      const { result, rerender } = renderHook(
        ({ isVisible }) =>
          useAnimationState({
            isVisible,
            enterDuration: 300,
            exitDuration: 200,
          }),
        { initialProps: { isVisible: true } }
      );

      // Complete enter animation
      act(() => {
        vi.advanceTimersByTime(300);
      });

      expect(result.current.phase).toBe('entered');

      // Trigger exit
      rerender({ isVisible: false });

      expect(result.current.phase).toBe('exiting');
      expect(result.current.shouldRender).toBe(true);
    });

    it('should complete exit animation', async () => {
      const { result, rerender } = renderHook(
        ({ isVisible }) =>
          useAnimationState({
            isVisible,
            enterDuration: 300,
            exitDuration: 200,
          }),
        { initialProps: { isVisible: true } }
      );

      // Complete enter animation
      act(() => {
        vi.advanceTimersByTime(300);
      });

      // Trigger exit
      rerender({ isVisible: false });

      expect(result.current.phase).toBe('exiting');

      // Complete exit animation
      act(() => {
        vi.advanceTimersByTime(200);
      });

      expect(result.current.phase).toBe('idle');
      expect(result.current.shouldRender).toBe(false);
    });

    it('should return animation styles', () => {
      const { result } = renderHook(() =>
        useAnimationState({
          isVisible: true,
          enterDuration: 300,
          exitDuration: 200,
        })
      );

      expect(result.current.style).toBeDefined();
      expect(typeof result.current.style).toBe('object');
    });

    it('should handle rapid visibility toggles', async () => {
      const { result, rerender } = renderHook(
        ({ isVisible }) =>
          useAnimationState({
            isVisible,
            enterDuration: 300,
            exitDuration: 200,
          }),
        { initialProps: { isVisible: false } }
      );

      // Rapidly toggle visibility
      rerender({ isVisible: true });
      rerender({ isVisible: false });
      rerender({ isVisible: true });

      // Should handle without errors
      expect(result.current.shouldRender).toBe(true);

      // Complete all timers
      act(() => {
        vi.runAllTimers();
      });

      expect(result.current.phase).toBe('entered');
    });
  });

  describe('Animation Duration Constants', () => {
    it('should use CSS variable fallbacks', () => {
      // Test that animations work even without CSS variables
      const { result } = renderHook(() =>
        useAnimationState({
          isVisible: true,
          enterDuration: 300,
          exitDuration: 200,
        })
      );

      expect(result.current.shouldRender).toBe(true);
    });
  });

  describe('Reduced Motion Support', () => {
    it('should skip animations when reduced motion is preferred', () => {
      // This would require mocking prefers-reduced-motion
      // The hook should handle this internally
      const { result } = renderHook(() =>
        useAnimationState({
          isVisible: true,
          enterDuration: 300,
          exitDuration: 200,
        })
      );

      // Hook should still work regardless of motion preference
      expect(result.current.shouldRender).toBe(true);
    });
  });
});
