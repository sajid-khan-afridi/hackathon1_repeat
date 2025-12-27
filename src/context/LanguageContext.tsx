import React, { createContext, useContext, useCallback, useEffect, ReactNode } from 'react';
import type { LanguageCode, TextDirection, LanguageConfig } from '../types/translation';
import { LANGUAGE_CONFIGS, LANGUAGE_PREFERENCE_KEY, DEFAULT_LANGUAGE } from '../types/translation';

interface LanguageContextType {
  /** Current language code */
  language: LanguageCode;
  /** Current text direction */
  direction: TextDirection;
  /** Current language configuration */
  config: LanguageConfig;
  /** Set the language */
  setLanguage: (lang: LanguageCode) => void;
  /** Whether a translation error has occurred */
  hasTranslationError: boolean;
  /** Set translation error state */
  setTranslationError: (hasError: boolean) => void;
  /** Whether the system is loading */
  isLoading: boolean;
}

const VALID_LANGUAGES: LanguageCode[] = ['en', 'ur'];

const defaultLanguageContext: LanguageContextType = {
  language: DEFAULT_LANGUAGE,
  direction: 'ltr',
  config: LANGUAGE_CONFIGS[DEFAULT_LANGUAGE],
  setLanguage: () => {},
  hasTranslationError: false,
  setTranslationError: () => {},
  isLoading: true,
};

export const LanguageContext = createContext<LanguageContextType>(defaultLanguageContext);

interface LanguageProviderProps {
  children: ReactNode;
}

/**
 * Safely read language from localStorage with validation
 */
function loadFromStorage(): LanguageCode {
  if (typeof window === 'undefined') {
    return DEFAULT_LANGUAGE;
  }
  try {
    const saved = localStorage.getItem(LANGUAGE_PREFERENCE_KEY);
    if (saved) {
      const parsed = JSON.parse(saved);
      if (parsed?.language && VALID_LANGUAGES.includes(parsed.language)) {
        return parsed.language as LanguageCode;
      }
    }
  } catch (error) {
    // Try legacy format (plain string)
    try {
      const saved = localStorage.getItem('language');
      if (saved && VALID_LANGUAGES.includes(saved as LanguageCode)) {
        return saved as LanguageCode;
      }
    } catch {
      console.warn('Failed to load language from localStorage:', error);
    }
  }
  return DEFAULT_LANGUAGE;
}

/**
 * Safely write language to localStorage with timestamp
 */
function saveToStorage(lang: LanguageCode): void {
  if (typeof window === 'undefined') {
    return;
  }
  try {
    const preference = {
      language: lang,
      updatedAt: new Date().toISOString(),
    };
    localStorage.setItem(LANGUAGE_PREFERENCE_KEY, JSON.stringify(preference));
  } catch (error) {
    console.warn('Failed to save language to localStorage:', error);
  }
}

/**
 * Update document direction and lang attributes
 */
function updateDocumentDirection(lang: LanguageCode): void {
  if (typeof document === 'undefined') {
    return;
  }
  const config = LANGUAGE_CONFIGS[lang];
  document.documentElement.setAttribute('dir', config.direction);
  document.documentElement.setAttribute('lang', config.htmlLang);
}

/**
 * Announce language change to screen readers
 */
function announceLanguageChange(lang: LanguageCode): void {
  if (typeof document === 'undefined') {
    return;
  }

  const config = LANGUAGE_CONFIGS[lang];
  const announcement = lang === 'ur'
    ? 'زبان اردو میں تبدیل کی گئی'
    : `Language changed to ${config.label}`;

  // Create or update aria-live region
  let liveRegion = document.getElementById('language-announcer');
  if (!liveRegion) {
    liveRegion = document.createElement('div');
    liveRegion.id = 'language-announcer';
    liveRegion.setAttribute('aria-live', 'polite');
    liveRegion.setAttribute('aria-atomic', 'true');
    liveRegion.style.cssText = 'position: absolute; left: -10000px; width: 1px; height: 1px; overflow: hidden;';
    document.body.appendChild(liveRegion);
  }

  // Clear and set new announcement
  liveRegion.textContent = '';
  setTimeout(() => {
    liveRegion!.textContent = announcement;
  }, 100);
}

export function LanguageProvider({ children }: LanguageProviderProps) {
  const [language, setLanguageState] = React.useState<LanguageCode>(DEFAULT_LANGUAGE);
  const [hasTranslationError, setTranslationError] = React.useState(false);
  const [isLoading, setIsLoading] = React.useState(true);

  // Load saved language on mount
  useEffect(() => {
    const savedLang = loadFromStorage();
    setLanguageState(savedLang);
    updateDocumentDirection(savedLang);
    setIsLoading(false);
  }, []);

  // Memoized setter that validates, persists, and updates document
  const setLanguage = useCallback((lang: LanguageCode) => {
    if (!VALID_LANGUAGES.includes(lang)) {
      console.warn(`Invalid language: ${lang}. Defaulting to ${DEFAULT_LANGUAGE}`);
      lang = DEFAULT_LANGUAGE;
    }

    const previousLang = language;
    setLanguageState(lang);
    saveToStorage(lang);
    updateDocumentDirection(lang);

    // Announce change if language actually changed
    if (previousLang !== lang) {
      announceLanguageChange(lang);
      // Clear any previous translation errors on language change
      setTranslationError(false);
    }
  }, [language]);

  const config = LANGUAGE_CONFIGS[language];

  return (
    <LanguageContext.Provider
      value={{
        language,
        direction: config.direction,
        config,
        setLanguage,
        hasTranslationError,
        setTranslationError,
        isLoading,
      }}
    >
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguageContext() {
  return useContext(LanguageContext);
}

// Re-export types for convenience
export type { LanguageCode, TextDirection, LanguageConfig };
