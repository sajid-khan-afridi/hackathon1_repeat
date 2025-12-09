import { useState, useEffect, useCallback } from 'react';
import { KeywordExtractor } from '../keywordExtractor.js';

export interface ChipData {
  text: string;
  type: 'entity' | 'noun' | 'action' | 'question' | 'tech' | 'proper' | 'adjective';
  score: number;
}

export interface UseKeywordExtractionOptions {
  maxChips?: number;
  minWordLength?: number;
  excludeWords?: string[];
  debounceMs?: number;
}

export const useKeywordExtraction = (
  text: string,
  options: UseKeywordExtractionOptions = {}
) => {
  const [keywords, setKeywords] = useState<ChipData[]>([]);
  const [categories, setCategories] = useState<Record<string, ChipData[]>>({});
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const {
    maxChips = 5,
    minWordLength = 3,
    excludeWords = [],
    debounceMs = 300
  } = options;

  const extractKeywords = useCallback((inputText: string) => {
    if (!inputText || typeof inputText !== 'string') {
      setKeywords([]);
      setCategories({});
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const extractor = new KeywordExtractor({
        maxChips,
        minWordLength,
        excludeWords: new Set(excludeWords)
      });

      setTimeout(() => {
        const extractedKeywords = extractor.extractKeywords(inputText);
        const categorizedKeywords = extractor.categorizeChips(extractedKeywords);

        setKeywords(extractedKeywords);
        setCategories(categorizedKeywords);
        setIsLoading(false);
      }, debounceMs);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to extract keywords');
      setIsLoading(false);
    }
  }, [maxChips, minWordLength, excludeWords, debounceMs]);

  useEffect(() => {
    extractKeywords(text);
  }, [text, extractKeywords]);

  return {
    keywords,
    categories,
    isLoading,
    error,
    refetch: () => extractKeywords(text)
  };
};