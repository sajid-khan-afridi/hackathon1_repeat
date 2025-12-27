/**
 * Unit Tests: useChatPanelState Hook
 *
 * Tests the chat panel state management hook.
 */

import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { renderHook, act } from '@testing-library/react';
import { useChatPanelState, PANEL_WIDTH_CONSTRAINTS } from '../../src/hooks/useChatPanelState';

describe('useChatPanelState', () => {
  let localStorageMock: Record<string, string>;
  let sessionStorageMock: Record<string, string>;

  beforeEach(() => {
    localStorageMock = {};
    sessionStorageMock = {};

    Object.defineProperty(window, 'localStorage', {
      value: {
        getItem: (key: string) => localStorageMock[key] ?? null,
        setItem: (key: string, value: string) => {
          localStorageMock[key] = value;
        },
        removeItem: (key: string) => {
          delete localStorageMock[key];
        },
        clear: () => {
          localStorageMock = {};
        },
      },
      writable: true,
    });

    Object.defineProperty(window, 'sessionStorage', {
      value: {
        getItem: (key: string) => sessionStorageMock[key] ?? null,
        setItem: (key: string, value: string) => {
          sessionStorageMock[key] = value;
        },
        removeItem: (key: string) => {
          delete sessionStorageMock[key];
        },
        clear: () => {
          sessionStorageMock = {};
        },
      },
      writable: true,
    });
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  it('should initialize with default values', () => {
    const { result } = renderHook(() => useChatPanelState());

    expect(result.current.isOpen).toBe(false);
    expect(result.current.isMinimized).toBe(false);
    expect(result.current.width).toBe(PANEL_WIDTH_CONSTRAINTS.DEFAULT);
    expect(result.current.unreadCount).toBe(0);
  });

  it('should toggle open state', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.open();
    });

    expect(result.current.isOpen).toBe(true);

    act(() => {
      result.current.close();
    });

    expect(result.current.isOpen).toBe(false);
  });

  it('should toggle minimize state', () => {
    const { result } = renderHook(() => useChatPanelState());

    // Open first
    act(() => {
      result.current.open();
    });

    act(() => {
      result.current.minimize();
    });

    expect(result.current.isMinimized).toBe(true);

    act(() => {
      result.current.expand();
    });

    expect(result.current.isMinimized).toBe(false);
  });

  it('should update width within constraints', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.setWidth(500);
    });

    expect(result.current.width).toBe(500);
  });

  it('should clamp width to minimum', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.setWidth(100);
    });

    expect(result.current.width).toBe(PANEL_WIDTH_CONSTRAINTS.MIN);
  });

  it('should clamp width to maximum', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.setWidth(1000);
    });

    expect(result.current.width).toBe(PANEL_WIDTH_CONSTRAINTS.MAX);
  });

  it('should increment unread count', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.incrementUnread();
    });

    expect(result.current.unreadCount).toBe(1);

    act(() => {
      result.current.incrementUnread();
    });

    expect(result.current.unreadCount).toBe(2);
  });

  it('should reset unread count', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.incrementUnread();
      result.current.incrementUnread();
      result.current.resetUnread();
    });

    expect(result.current.unreadCount).toBe(0);
  });

  it('should persist width to localStorage', () => {
    const { result } = renderHook(() => useChatPanelState());

    act(() => {
      result.current.setWidth(450);
    });

    // Check localStorage was updated
    expect(localStorageMock).toHaveProperty('chatPanelWidth');
  });

  it('should restore width from localStorage', () => {
    localStorageMock['chatPanelWidth'] = '500';

    const { result } = renderHook(() => useChatPanelState());

    expect(result.current.width).toBe(500);
  });

  it('should handle localStorage errors gracefully', () => {
    // Simulate localStorage throwing
    Object.defineProperty(window, 'localStorage', {
      value: {
        getItem: () => {
          throw new Error('Storage quota exceeded');
        },
        setItem: () => {
          throw new Error('Storage quota exceeded');
        },
      },
      writable: true,
    });

    // Should not throw
    const { result } = renderHook(() => useChatPanelState());

    expect(result.current.width).toBe(PANEL_WIDTH_CONSTRAINTS.DEFAULT);
  });
});
