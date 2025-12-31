/**
 * useLanguagePreference Hook
 * Provides language preference management with localStorage persistence
 * Phase 5: Translation Feature - T008
 */

import { useCallback, useEffect, useState } from 'react';
import type { LanguageCode, LanguagePreference } from '../types/translation';
import { LANGUAGE_PREFERENCE_KEY, DEFAULT_LANGUAGE, LANGUAGE_CONFIGS } from '../types/translation';

interface UseLanguagePreferenceReturn {
  /** Current language code */
  language: LanguageCode;
  /** Current text direction */
  direction: 'ltr' | 'rtl';
  /** Whether preference is loading */
  isLoading: boolean;
  /** Error if preference failed to load/save */
  error: Error | null;
  /** Set the language preference */
  setLanguage: (lang: LanguageCode) => void;
  /** Clear the stored preference */
  clearPreference: () => void;
  /** Get the full preference object */
  getPreference: () => LanguagePreference | null;
}

/**
 * Hook for managing language preferences with localStorage persistence
 *
 * @example
 * ```tsx
 * const { language, direction, setLanguage } = useLanguagePreference();
 *
 * return (
 *   <div dir={direction}>
 *     <button onClick={() => setLanguage('ur')}>Switch to Urdu</button>
 *   </div>
 * );
 * ```
 */
export function useLanguagePreference(): UseLanguagePreferenceReturn {
  const [language, setLanguageState] = useState<LanguageCode>(DEFAULT_LANGUAGE);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  // Load preference from localStorage on mount
  useEffect(() => {
    if (typeof window === 'undefined') {
      setIsLoading(false);
      return;
    }

    try {
      const stored = localStorage.getItem(LANGUAGE_PREFERENCE_KEY);
      if (stored) {
        const preference: LanguagePreference = JSON.parse(stored);
        if (preference.language && ['en', 'ur'].includes(preference.language)) {
          setLanguageState(preference.language);
        }
      }
    } catch (err) {
      // Try legacy format (plain string)
      try {
        const legacy = localStorage.getItem('language');
        if (legacy && ['en', 'ur'].includes(legacy)) {
          setLanguageState(legacy as LanguageCode);
          // Migrate to new format
          savePreference(legacy as LanguageCode);
        }
      } catch {
        setError(err instanceof Error ? err : new Error('Failed to load language preference'));
      }
    } finally {
      setIsLoading(false);
    }
  }, []);

  // Save preference to localStorage
  const savePreference = useCallback((lang: LanguageCode): void => {
    if (typeof window === 'undefined') return;

    try {
      const preference: LanguagePreference = {
        language: lang,
        updatedAt: new Date().toISOString(),
      };
      localStorage.setItem(LANGUAGE_PREFERENCE_KEY, JSON.stringify(preference));
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err : new Error('Failed to save language preference'));
    }
  }, []);

  // Set language with validation and persistence
  const setLanguage = useCallback(
    (lang: LanguageCode): void => {
      if (!['en', 'ur'].includes(lang)) {
        console.warn(`Invalid language: ${lang}. Defaulting to ${DEFAULT_LANGUAGE}`);
        lang = DEFAULT_LANGUAGE;
      }

      setLanguageState(lang);
      savePreference(lang);
    },
    [savePreference]
  );

  // Clear stored preference
  const clearPreference = useCallback((): void => {
    if (typeof window === 'undefined') return;

    try {
      localStorage.removeItem(LANGUAGE_PREFERENCE_KEY);
      localStorage.removeItem('language'); // Also clear legacy key
      setLanguageState(DEFAULT_LANGUAGE);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err : new Error('Failed to clear language preference'));
    }
  }, []);

  // Get the full preference object
  const getPreference = useCallback((): LanguagePreference | null => {
    if (typeof window === 'undefined') return null;

    try {
      const stored = localStorage.getItem(LANGUAGE_PREFERENCE_KEY);
      if (stored) {
        return JSON.parse(stored) as LanguagePreference;
      }
    } catch {
      // Ignore parse errors
    }
    return null;
  }, []);

  const config = LANGUAGE_CONFIGS[language];

  return {
    language,
    direction: config.direction,
    isLoading,
    error,
    setLanguage,
    clearPreference,
    getPreference,
  };
}

export default useLanguagePreference;
