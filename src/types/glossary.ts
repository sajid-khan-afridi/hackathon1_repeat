/**
 * Glossary Types for Bilingual Technical Terms
 * Phase 5: Translation Feature
 */

/**
 * Categories for organizing glossary terms
 * Using string union for flexibility with JSON data
 */
export type GlossaryCategory =
  | 'Robotics Frameworks'
  | 'Programming Languages'
  | 'Hardware Components'
  | 'AI/ML Concepts'
  | 'Mathematics'
  | 'Simulation'
  | 'Middleware'
  | 'Standards'
  | null;

/**
 * Represents a single technical term in the bilingual glossary
 */
export interface GlossaryTerm {
  /**
   * Unique identifier (kebab-case, e.g., "ros-2")
   */
  id: string;

  /**
   * English term (e.g., "ROS 2")
   */
  english: string;

  /**
   * Urdu script transliteration
   */
  urduTransliteration: string;

  /**
   * English definition (1-2 sentences)
   */
  definition: string;

  /**
   * Urdu definition
   */
  definitionUrdu: string;

  /**
   * Category for filtering
   */
  category: GlossaryCategory;

  /**
   * Array of related term IDs (optional)
   */
  relatedTerms?: string[];
}

/**
 * Structure of the glossary.json file
 */
export interface GlossaryData {
  /**
   * Version of the glossary schema
   */
  version: string;

  /**
   * Last updated timestamp (ISO 8601)
   */
  lastUpdated: string;

  /**
   * Array of glossary terms
   */
  terms: GlossaryTerm[];
}

/**
 * Props for GlossaryTooltip component
 */
export interface GlossaryTooltipProps {
  /**
   * The term ID to display tooltip for
   */
  termId: string;

  /**
   * Children to wrap with tooltip
   */
  children: React.ReactNode;
}

/**
 * Props for GlossaryPage component
 */
export interface GlossaryPageProps {
  /**
   * Initial search query (optional)
   */
  initialQuery?: string;

  /**
   * Initial category filter (optional)
   */
  initialCategory?: GlossaryCategory;
}
