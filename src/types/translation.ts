/**
 * Translation Types for Phase 5: Translation Feature
 * Defines types for translation cache, language preferences, and related data
 */

/**
 * Supported language codes
 */
export type LanguageCode = 'en' | 'ur';

/**
 * Language direction
 */
export type TextDirection = 'ltr' | 'rtl';

/**
 * Language configuration
 */
export interface LanguageConfig {
  code: LanguageCode;
  label: string;
  direction: TextDirection;
  htmlLang: string;
}

/**
 * Available language configurations
 */
export const LANGUAGE_CONFIGS: Record<LanguageCode, LanguageConfig> = {
  en: {
    code: 'en',
    label: 'English',
    direction: 'ltr',
    htmlLang: 'en-US',
  },
  ur: {
    code: 'ur',
    label: 'اردو',
    direction: 'rtl',
    htmlLang: 'ur-PK',
  },
};

/**
 * Translation cache metadata (stored alongside translated MDX files)
 */
export interface TranslationCache {
  /**
   * Chapter identifier (e.g., "module-1/chapter-1")
   */
  chapterId: string;

  /**
   * MD5 hash of source English content for cache invalidation
   */
  sourceHash: string;

  /**
   * ISO 8601 timestamp of when translation was generated
   */
  translatedAt: string;

  /**
   * Path to source MDX file
   */
  sourceFile: string;

  /**
   * Path to translated MDX file
   */
  translatedFile: string;

  /**
   * Number of technical terms preserved in English
   */
  termCount: number;
}

/**
 * User's language preference (stored in localStorage)
 */
export interface LanguagePreference {
  /**
   * Selected language code
   */
  language: LanguageCode;

  /**
   * ISO 8601 timestamp of last update
   */
  updatedAt: string;
}

/**
 * Translation status for a chapter
 */
export type TranslationStatus = 'available' | 'pending' | 'error' | 'stale';

/**
 * Chapter translation metadata
 */
export interface ChapterTranslation {
  /**
   * Chapter identifier
   */
  chapterId: string;

  /**
   * Current translation status
   */
  status: TranslationStatus;

  /**
   * Cache metadata if available
   */
  cache?: TranslationCache;

  /**
   * Error message if status is 'error'
   */
  errorMessage?: string;
}

/**
 * Translation service result
 */
export interface TranslationResult {
  /**
   * Whether the translation was successful
   */
  success: boolean;

  /**
   * Translated content (if successful)
   */
  content?: string;

  /**
   * Error message (if failed)
   */
  error?: string;

  /**
   * Cache metadata
   */
  cache?: TranslationCache;
}

/**
 * Technical term protection markers
 */
export interface TermProtection {
  /**
   * Original term in English
   */
  original: string;

  /**
   * Placeholder used during translation
   */
  placeholder: string;

  /**
   * Position in the content (start index)
   */
  position: number;
}

/**
 * LocalStorage key for language preference
 */
export const LANGUAGE_PREFERENCE_KEY = 'hackathon1_language_preference';

/**
 * Default language code
 */
export const DEFAULT_LANGUAGE: LanguageCode = 'en';
