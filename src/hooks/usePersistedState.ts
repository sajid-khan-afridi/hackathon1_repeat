/**
 * usePersistedState Hook
 *
 * Provides state that is automatically persisted to localStorage or sessionStorage.
 * Includes fallback to in-memory storage when browser storage is unavailable.
 *
 * @see specs/007-enhance-ui/data-model.md for interface definitions
 */

import { useState, useEffect, useCallback } from 'react';

/**
 * Options for usePersistedState hook
 */
export interface UsePersistedStateOptions<T> {
  /** Storage type: 'localStorage' persists across sessions, 'sessionStorage' clears on tab close */
  storage: 'localStorage' | 'sessionStorage';

  /** Custom serialization function (defaults to JSON.stringify) */
  serialize?: (value: T) => string;

  /** Custom deserialization function (defaults to JSON.parse) */
  deserialize?: (value: string) => T;
}

/**
 * Return type matches React's useState
 */
export type UsePersistedStateReturn<T> = [T, (value: T | ((prev: T) => T)) => void];

// In-memory fallback storage for when browser storage is unavailable
const memoryStorage: Map<string, string> = new Map();

/**
 * Checks if browser storage is available
 */
function isStorageAvailable(type: 'localStorage' | 'sessionStorage'): boolean {
  if (typeof window === 'undefined') {
    return false;
  }

  try {
    const storage = window[type];
    const testKey = '__storage_test__';
    storage.setItem(testKey, testKey);
    storage.removeItem(testKey);
    return true;
  } catch {
    return false;
  }
}

/**
 * Custom hook that persists state to browser storage with automatic fallback.
 *
 * Features:
 * - Automatic fallback to in-memory storage when browser storage is unavailable
 * - SSR-safe (works with server-side rendering)
 * - Custom serialization/deserialization support
 * - Type-safe with TypeScript generics
 *
 * @param key - The storage key
 * @param defaultValue - The default value if no stored value exists
 * @param options - Configuration options
 *
 * @example
 * ```tsx
 * // Persist panel width to localStorage
 * const [width, setWidth] = usePersistedState(
 *   'chat-panel-width',
 *   400,
 *   { storage: 'localStorage' }
 * );
 *
 * // Persist minimize state to sessionStorage
 * const [isMinimized, setIsMinimized] = usePersistedState(
 *   'chat-panel-minimized',
 *   false,
 *   { storage: 'sessionStorage' }
 * );
 * ```
 */
export function usePersistedState<T>(
  key: string,
  defaultValue: T,
  options: UsePersistedStateOptions<T>
): UsePersistedStateReturn<T> {
  const { storage, serialize = JSON.stringify, deserialize = JSON.parse } = options;

  // Check if storage is available
  const storageAvailable = isStorageAvailable(storage);

  // Initialize state with stored value or default
  const [state, setState] = useState<T>(() => {
    // SSR check
    if (typeof window === 'undefined') {
      return defaultValue;
    }

    try {
      let storedValue: string | null = null;

      if (storageAvailable) {
        storedValue = window[storage].getItem(key);
      } else {
        storedValue = memoryStorage.get(key) ?? null;
      }

      if (storedValue !== null) {
        return deserialize(storedValue);
      }
    } catch (error) {
      console.warn(`usePersistedState: Failed to read key "${key}":`, error);
    }

    return defaultValue;
  });

  // Persist state changes
  useEffect(() => {
    // SSR check
    if (typeof window === 'undefined') {
      return;
    }

    try {
      const serializedValue = serialize(state);

      if (storageAvailable) {
        window[storage].setItem(key, serializedValue);
      } else {
        memoryStorage.set(key, serializedValue);
      }
    } catch (error) {
      console.warn(`usePersistedState: Failed to write key "${key}":`, error);
    }
  }, [key, state, storage, storageAvailable, serialize]);

  // Enhanced setter that supports functional updates
  const setPersistedState = useCallback((value: T | ((prev: T) => T)) => {
    setState((prevState) => {
      const newState = typeof value === 'function' ? (value as (prev: T) => T)(prevState) : value;
      return newState;
    });
  }, []);

  return [state, setPersistedState];
}

/**
 * Clears a persisted state value from storage
 */
export function clearPersistedState(key: string, storage: 'localStorage' | 'sessionStorage'): void {
  if (typeof window === 'undefined') {
    return;
  }

  try {
    if (isStorageAvailable(storage)) {
      window[storage].removeItem(key);
    } else {
      memoryStorage.delete(key);
    }
  } catch (error) {
    console.warn(`clearPersistedState: Failed to clear key "${key}":`, error);
  }
}

export default usePersistedState;
