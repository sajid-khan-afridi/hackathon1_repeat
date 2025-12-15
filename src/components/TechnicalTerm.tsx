import React, { useMemo } from 'react';
import { useLanguageContext } from '@site/src/context/LanguageContext';
import termsGlossary from '@site/src/data/technical-terms.json';

interface TechnicalTermProps {
  term: string;
  explanation?: string;
}

interface TermData {
  term: string;
  category: string;
  translationStatus: string;
  contextualExplanation?: string;
}

/**
 * Component for marking technical terms that should be preserved during translation.
 * Includes accessibility features for screen readers and keyboard navigation.
 *
 * Usage:
 * <TechnicalTerm term="ROS 2" explanation="Robot Operating System 2" />
 * <TechnicalTerm term="publisher" />
 */
export default function TechnicalTerm({
  term,
  explanation,
}: TechnicalTermProps): React.ReactElement {
  const { language } = useLanguageContext();

  // Generate a stable ID for accessibility linking
  const termId = useMemo(
    () =>
      `term-${term.toLowerCase().replace(/\s+/g, '-')}-${Math.random().toString(36).substr(2, 9)}`,
    [term]
  );

  // Find term data from glossary (memoized for performance)
  const termData = useMemo(
    () => (termsGlossary as { terms: TermData[] }).terms.find((t) => t.term === term),
    [term]
  );

  // For English, only use explicit explanation prop
  // For Urdu, use contextual explanation from glossary
  const termExplanation =
    language === 'ur' ? explanation || termData?.contextualExplanation || '' : explanation || '';

  // For Urdu translation, show term with contextual explanation
  if (language === 'ur' && termData?.contextualExplanation) {
    return (
      <span className="technical-term" role="term" aria-describedby={termId} tabIndex={0}>
        <span className="term-original" lang="en">
          {term}
        </span>
        <span id={termId} className="term-transliteration" lang="ur" role="definition">
          ({termData.contextualExplanation})
        </span>
      </span>
    );
  }

  // For English or other languages, show the term with tooltip
  return (
    <span
      className="technical-term"
      role="term"
      title={termExplanation}
      aria-label={termExplanation ? `${term}: ${termExplanation}` : term}
      tabIndex={0}
    >
      {term}
      {/* Hidden definition for screen readers */}
      {termExplanation && (
        <span id={termId} className="sr-only" role="definition">
          {termExplanation}
        </span>
      )}
    </span>
  );
}
