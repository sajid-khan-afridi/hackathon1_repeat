/**
 * useGlossary Hook
 * Provides access to the bilingual technical glossary with search and filter
 * Phase 5: Translation Feature - T037
 */

import { useState, useEffect, useMemo, useCallback } from 'react';
import type { GlossaryTerm, GlossaryData, GlossaryCategory } from '../types/glossary';

// Import glossary data
import glossaryData from '../data/glossary.json';

interface UseGlossaryReturn {
  /** All glossary terms */
  terms: GlossaryTerm[];
  /** Filtered terms based on search and category */
  filteredTerms: GlossaryTerm[];
  /** Current search query */
  searchQuery: string;
  /** Set search query */
  setSearchQuery: (query: string) => void;
  /** Selected category filter */
  categoryFilter: GlossaryCategory | null;
  /** Set category filter */
  setCategoryFilter: (category: GlossaryCategory | null) => void;
  /** Available categories */
  categories: GlossaryCategory[];
  /** Get a term by ID */
  getTermById: (id: string) => GlossaryTerm | undefined;
  /** Get related terms for a term */
  getRelatedTerms: (termId: string) => GlossaryTerm[];
  /** Check if a term exists */
  hasTerm: (id: string) => boolean;
  /** Loading state */
  isLoading: boolean;
  /** Error state */
  error: Error | null;
}

/**
 * Hook for accessing and searching the bilingual glossary
 *
 * @example
 * ```tsx
 * const { filteredTerms, searchQuery, setSearchQuery, setCategoryFilter } = useGlossary();
 *
 * return (
 *   <div>
 *     <input
 *       value={searchQuery}
 *       onChange={(e) => setSearchQuery(e.target.value)}
 *       placeholder="Search terms..."
 *     />
 *     <ul>
 *       {filteredTerms.map((term) => (
 *         <li key={term.id}>{term.english} - {term.urduTransliteration}</li>
 *       ))}
 *     </ul>
 *   </div>
 * );
 * ```
 */
export function useGlossary(): UseGlossaryReturn {
  const [searchQuery, setSearchQuery] = useState('');
  const [categoryFilter, setCategoryFilter] = useState<GlossaryCategory | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  // Parse glossary data
  const terms = useMemo<GlossaryTerm[]>(() => {
    try {
      return (glossaryData as GlossaryData).terms;
    } catch (err) {
      setError(err instanceof Error ? err : new Error('Failed to parse glossary data'));
      return [];
    }
  }, []);

  // Extract unique categories
  const categories = useMemo<GlossaryCategory[]>(() => {
    const categorySet = new Set(terms.map((term) => term.category));
    return Array.from(categorySet).sort() as GlossaryCategory[];
  }, [terms]);

  // Set loading to false after initial render
  useEffect(() => {
    setIsLoading(false);
  }, []);

  // Filter terms based on search query and category
  const filteredTerms = useMemo<GlossaryTerm[]>(() => {
    let result = terms;

    // Filter by category
    if (categoryFilter) {
      result = result.filter((term) => term.category === categoryFilter);
    }

    // Filter by search query
    if (searchQuery.trim()) {
      const query = searchQuery.toLowerCase().trim();
      result = result.filter(
        (term) =>
          term.english.toLowerCase().includes(query) ||
          term.urduTransliteration.includes(query) ||
          term.definition.toLowerCase().includes(query) ||
          term.definitionUrdu.includes(query)
      );
    }

    return result;
  }, [terms, searchQuery, categoryFilter]);

  // Get term by ID
  const getTermById = useCallback(
    (id: string): GlossaryTerm | undefined => {
      return terms.find((term) => term.id === id);
    },
    [terms]
  );

  // Get related terms
  const getRelatedTerms = useCallback(
    (termId: string): GlossaryTerm[] => {
      const term = getTermById(termId);
      if (!term || !term.relatedTerms) {
        return [];
      }
      return term.relatedTerms
        .map((id) => getTermById(id))
        .filter((t): t is GlossaryTerm => t !== undefined);
    },
    [getTermById]
  );

  // Check if term exists
  const hasTerm = useCallback(
    (id: string): boolean => {
      return terms.some((term) => term.id === id);
    },
    [terms]
  );

  return {
    terms,
    filteredTerms,
    searchQuery,
    setSearchQuery,
    categoryFilter,
    setCategoryFilter,
    categories,
    getTermById,
    getRelatedTerms,
    hasTerm,
    isLoading,
    error,
  };
}

export default useGlossary;
