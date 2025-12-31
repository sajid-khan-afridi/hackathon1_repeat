/**
 * GlossaryPage Component
 * Searchable bilingual glossary page with category filtering
 * Phase 5: Translation Feature - T040
 */

import React from 'react';
import { useGlossary } from '../../hooks/useGlossary';
import { useLanguageContext } from '../../context/LanguageContext';
import type { GlossaryCategory } from '../../types/glossary';
import styles from './GlossaryPage.module.css';

export interface GlossaryPageProps {
  /** Initial search query */
  initialQuery?: string;
  /** Initial category filter */
  initialCategory?: GlossaryCategory;
}

/**
 * GlossaryPage displays a searchable, filterable list of technical terms
 * with bilingual definitions (English and Urdu).
 */
export function GlossaryPage({
  initialQuery = '',
  initialCategory = null,
}: GlossaryPageProps): React.ReactElement {
  const {
    filteredTerms,
    searchQuery,
    setSearchQuery,
    categoryFilter,
    setCategoryFilter,
    categories,
    isLoading,
    error,
  } = useGlossary();

  const { language } = useLanguageContext();

  // Initialize with props
  React.useEffect(() => {
    if (initialQuery) {
      setSearchQuery(initialQuery);
    }
    if (initialCategory) {
      setCategoryFilter(initialCategory);
    }
  }, [initialQuery, initialCategory, setSearchQuery, setCategoryFilter]);

  const handleSearchChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSearchQuery(event.target.value);
  };

  const handleCategoryChange = (event: React.ChangeEvent<HTMLSelectElement>) => {
    const value = event.target.value;
    setCategoryFilter(value ? (value as GlossaryCategory) : null);
  };

  const handleClearFilters = () => {
    setSearchQuery('');
    setCategoryFilter(null);
  };

  // Labels based on language
  const labels = {
    title: language === 'ur' ? 'تکنیکی اصطلاحات' : 'Technical Glossary',
    subtitle:
      language === 'ur'
        ? 'روبوٹکس اور AI کی دو لسانی اصطلاحات'
        : 'Bilingual robotics and AI terminology',
    searchPlaceholder: language === 'ur' ? 'اصطلاحات تلاش کریں...' : 'Search terms...',
    allCategories: language === 'ur' ? 'تمام زمرے' : 'All categories',
    clearFilters: language === 'ur' ? 'فلٹرز صاف کریں' : 'Clear filters',
    noResults: language === 'ur' ? 'کوئی اصطلاح نہیں ملی' : 'No terms found',
    termsCount:
      language === 'ur' ? `${filteredTerms.length} اصطلاحات` : `${filteredTerms.length} terms`,
    definition: language === 'ur' ? 'تعریف' : 'Definition',
    relatedTerms: language === 'ur' ? 'متعلقہ اصطلاحات' : 'Related terms',
  };

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>
          <div className={styles.loadingSpinner} />
          <span>{language === 'ur' ? 'لوڈ ہو رہا ہے...' : 'Loading...'}</span>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          <span className={styles.errorIcon}>⚠️</span>
          <p>{language === 'ur' ? 'اصطلاحات لوڈ کرنے میں خرابی' : 'Error loading glossary'}</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <header className={styles.header}>
        <h1 className={styles.title}>{labels.title}</h1>
        <p className={styles.subtitle}>{labels.subtitle}</p>
      </header>

      <div className={styles.controls}>
        <div className={styles.searchContainer}>
          <label htmlFor="glossary-search" className={styles.visuallyHidden}>
            {labels.searchPlaceholder}
          </label>
          <input
            id="glossary-search"
            type="search"
            className={styles.searchInput}
            placeholder={labels.searchPlaceholder}
            value={searchQuery}
            onChange={handleSearchChange}
            aria-label={labels.searchPlaceholder}
          />
          <span className={styles.searchIcon} aria-hidden="true">
            <svg
              width="20"
              height="20"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
            >
              <circle cx="11" cy="11" r="8" />
              <line x1="21" y1="21" x2="16.65" y2="16.65" />
            </svg>
          </span>
        </div>

        <div className={styles.filterContainer}>
          <label htmlFor="glossary-category" className={styles.visuallyHidden}>
            {labels.allCategories}
          </label>
          <select
            id="glossary-category"
            className={styles.categorySelect}
            value={categoryFilter || ''}
            onChange={handleCategoryChange}
            aria-label={labels.allCategories}
          >
            <option value="">{labels.allCategories}</option>
            {categories
              .filter((c): c is NonNullable<typeof c> => c !== null)
              .map((category) => (
                <option key={category} value={category}>
                  {category}
                </option>
              ))}
          </select>
        </div>

        {(searchQuery || categoryFilter) && (
          <button type="button" className={styles.clearButton} onClick={handleClearFilters}>
            {labels.clearFilters}
          </button>
        )}
      </div>

      <div className={styles.resultsInfo}>
        <span>{labels.termsCount}</span>
      </div>

      {filteredTerms.length === 0 ? (
        <div className={styles.noResults}>
          <span className={styles.noResultsIcon}>🔍</span>
          <p>{labels.noResults}</p>
        </div>
      ) : (
        <ul className={styles.termsList} role="list">
          {filteredTerms.map((term) => (
            <li key={term.id} className={styles.termCard}>
              <div className={styles.termHeader}>
                <h2 className={styles.termEnglish}>{term.english}</h2>
                <span className={styles.termUrdu}>{term.urduTransliteration}</span>
              </div>

              <div className={styles.termBody}>
                <div className={styles.definitionBlock}>
                  <span className={styles.definitionLabel}>English:</span>
                  <p className={styles.definitionText}>{term.definition}</p>
                </div>

                <div className={styles.definitionBlock}>
                  <span className={styles.definitionLabel}>اردو:</span>
                  <p className={`${styles.definitionText} ${styles.urduText}`}>
                    {term.definitionUrdu}
                  </p>
                </div>
              </div>

              <div className={styles.termFooter}>
                <span className={styles.categoryBadge}>{term.category}</span>

                {term.relatedTerms && term.relatedTerms.length > 0 && (
                  <div className={styles.relatedTerms}>
                    <span className={styles.relatedLabel}>{labels.relatedTerms}:</span>
                    {term.relatedTerms.map((relatedId, index) => (
                      <span key={relatedId} className={styles.relatedTerm}>
                        {relatedId}
                        {index < term.relatedTerms!.length - 1 && ', '}
                      </span>
                    ))}
                  </div>
                )}
              </div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

export default GlossaryPage;
