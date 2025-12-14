import React, { createContext, useContext, useCallback, ReactNode } from 'react';

type Language = 'en' | 'ur';

interface LanguageContextType {
  language: Language;
  setLanguage: (lang: Language) => void;
}

const STORAGE_KEY = 'language';
const VALID_LANGUAGES: Language[] = ['en', 'ur'];
const DEFAULT_LANGUAGE: Language = 'en';

const defaultLanguageContext: LanguageContextType = {
  language: DEFAULT_LANGUAGE,
  setLanguage: () => {},
};

export const LanguageContext = createContext<LanguageContextType>(defaultLanguageContext);

interface LanguageProviderProps {
  children: ReactNode;
}

/**
 * Safely read language from localStorage with validation
 */
function loadFromStorage(): Language {
  if (typeof window === 'undefined') {
    return DEFAULT_LANGUAGE;
  }
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved && VALID_LANGUAGES.includes(saved as Language)) {
      return saved as Language;
    }
  } catch (error) {
    console.warn('Failed to load language from localStorage:', error);
  }
  return DEFAULT_LANGUAGE;
}

/**
 * Safely write language to localStorage
 */
function saveToStorage(lang: Language): void {
  if (typeof window === 'undefined') {
    return;
  }
  try {
    localStorage.setItem(STORAGE_KEY, lang);
  } catch (error) {
    console.warn('Failed to save language to localStorage:', error);
  }
}

export function LanguageProvider({ children }: LanguageProviderProps) {
  const [language, setLanguageState] = React.useState<Language>(loadFromStorage);

  // Memoized setter that validates and persists to localStorage
  const setLanguage = useCallback((lang: Language) => {
    if (!VALID_LANGUAGES.includes(lang)) {
      console.warn(`Invalid language: ${lang}. Defaulting to ${DEFAULT_LANGUAGE}`);
      lang = DEFAULT_LANGUAGE;
    }
    setLanguageState(lang);
    saveToStorage(lang);
  }, []);

  return (
    <LanguageContext.Provider value={{ language, setLanguage }}>
      {children}
    </LanguageContext.Provider>
  );
}

export function useLanguageContext() {
  return useContext(LanguageContext);
}
