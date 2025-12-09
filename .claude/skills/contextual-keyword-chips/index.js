/**
 * Contextual Keyword Chips Skill
 * Main entry point for the keyword chips functionality
 */

export { KeywordExtractor } from './keywordExtractor.js';

// Component exports (for React integration)
export {
  KeywordChips,
  useKeywordExtraction,
  QuickKeywordChips
} from './templates/KeywordChips.js';

export { useKeywordExtraction as useKeywordExtractionHook } from './templates/useKeywordExtraction.js';

// Utility function for direct usage
export function extractKeywordsFromText(text, options = {}) {
  const extractor = new KeywordExtractor(options);
  return extractor.extractKeywords(text);
}

// Example integration function
export function createKeywordChatIntegration({ onFollowUp, maxChips = 5 }) {
  return function KeywordChipsWrapper({ message, isAI }) {
    if (!isAI || !message) return null;

    return React.createElement(KeywordChips, {
      aiAnswer: message,
      onChipClick: (chipText) => {
        const followUpMessage = `Tell me more about: ${chipText}`;
        onFollowUp(followUpMessage);
      },
      maxChips,
      variant: 'default',
      animated: true
    });
  };
}

// Default export for easy importing
export default {
  KeywordExtractor,
  KeywordChips,
  useKeywordExtraction,
  QuickKeywordChips,
  extractKeywordsFromText,
  createKeywordChatIntegration
};