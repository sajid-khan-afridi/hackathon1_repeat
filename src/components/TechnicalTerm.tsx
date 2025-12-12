import React from 'react';
import { useLanguageContext } from '@site/src/context/LanguageContext';

interface TechnicalTermProps {
  term: string;
  explanation?: string;
}

/**
 * Component for marking technical terms that should be preserved during translation.
 *
 * Usage:
 * <TechnicalTerm term="ROS 2" explanation="Robot Operating System 2" />
 * <TechnicalTerm term="publisher" />
 */
export default function TechnicalTerm({
  term,
  explanation,
}: TechnicalTermProps): JSX.Element {
  const { language } = useLanguageContext();

  // For Urdu translation, show term with explanation
  if (language === 'ur') {
    const termsGlossary = require('@site/src/data/technical-terms.json');
    const termData = termsGlossary.terms.find((t: any) => t.term === term);

    if (termData?.contextualExplanation) {
      return (
        <span className="technical-term" title={explanation || termData.contextualExplanation}>
          <span className="term-original">{term}</span>
          <span className="term-transliteration">({termData.contextualExplanation})</span>
        </span>
      );
    }
  }

  // For English or other languages, just show the term
  return (
    <span className="technical-term" title={explanation}>
      {term}
    </span>
  );
}