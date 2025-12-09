import React, { useState, useEffect } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import clsx from 'clsx';
import { KeywordExtractor } from '../keywordExtractor.js';

interface ChipData {
  text: string;
  type: 'entity' | 'noun' | 'action' | 'question' | 'tech' | 'proper' | 'adjective';
  score: number;
}

interface KeywordChipsProps {
  aiAnswer: string;
  onChipClick?: (chipText: string) => void;
  maxChips?: number;
  className?: string;
  variant?: 'default' | 'compact' | 'expanded';
  showCategories?: boolean;
  animated?: boolean;
}

export const KeywordChips: React.FC<KeywordChipsProps> = ({
  aiAnswer,
  onChipClick,
  maxChips = 5,
  className,
  variant = 'default',
  showCategories = false,
  animated = true
}) => {
  const [chips, setChips] = useState<ChipData[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [categories, setCategories] = useState<Record<string, ChipData[]>>({});

  useEffect(() => {
    if (aiAnswer) {
      setIsLoading(true);
      const extractor = new KeywordExtractor({ maxChips });

      // Simulate async processing for better UX
      setTimeout(() => {
        const extractedChips = extractor.extractKeywords(aiAnswer);
        setChips(extractedChips);

        if (showCategories) {
          setCategories(extractor.categorizeChips(extractedChips));
        }
        setIsLoading(false);
      }, 300);
    }
  }, [aiAnswer, maxChips, showCategories]);

  const handleChipClick = (chip: ChipData) => {
    if (onChipClick) {
      onChipClick(chip.text);
    }
  };

  const getChipColor = (type: string) => {
    const colors = {
      entity: 'bg-purple-100 text-purple-800 hover:bg-purple-200 border-purple-300',
      noun: 'bg-blue-100 text-blue-800 hover:bg-blue-200 border-blue-300',
      action: 'bg-green-100 text-green-800 hover:bg-green-200 border-green-300',
      question: 'bg-orange-100 text-orange-800 hover:bg-orange-200 border-orange-300',
      tech: 'bg-indigo-100 text-indigo-800 hover:bg-indigo-200 border-indigo-300',
      proper: 'bg-gray-100 text-gray-800 hover:bg-gray-200 border-gray-300',
      adjective: 'bg-pink-100 text-pink-800 hover:bg-pink-200 border-pink-300'
    };
    return colors[type as keyof typeof colors] || colors.noun;
  };

  const getChipIcon = (type: string) => {
    const icons = {
      entity: '🏷️',
      noun: '💡',
      action: '⚡',
      question: '❓',
      tech: '⚙️',
      proper: '📍',
      adjective: '✨'
    };
    return icons[type as keyof typeof icons] || '💡';
  };

  const chipVariants = {
    default: 'px-4 py-2 text-sm',
    compact: 'px-3 py-1 text-xs',
    expanded: 'px-5 py-3 text-base'
  };

  const containerVariants = {
    hidden: { opacity: 0, y: 20 },
    visible: {
      opacity: 1,
      y: 0,
      transition: {
        staggerChildren: 0.1
      }
    }
  };

  const itemVariants = {
    hidden: { opacity: 0, scale: 0.8 },
    visible: {
      opacity: 1,
      scale: 1,
      transition: {
        type: 'spring',
        stiffness: 300,
        damping: 24
      }
    }
  };

  if (isLoading) {
    return (
      <div className="flex flex-wrap gap-2 animate-pulse">
        {Array.from({ length: 3 }).map((_, i) => (
          <div key={i} className="h-8 w-24 bg-gray-200 rounded-full" />
        ))}
      </div>
    );
  }

  if (chips.length === 0) {
    return null;
  }

  if (showCategories && Object.keys(categories).length > 0) {
    return (
      <motion.div
        initial="hidden"
        animate="visible"
        variants={containerVariants}
        className={clsx('space-y-3', className)}
      >
        {Object.entries(categories).map(([category, categoryChips]) => (
          categoryChips.length > 0 && (
            <div key={category}>
              <div className="text-xs font-medium text-gray-500 mb-2 capitalize">
                {category === 'tech' ? 'Technologies' : category}
              </div>
              <div className="flex flex-wrap gap-2">
                <AnimatePresence>
                  {categoryChips.map((chip, index) => (
                    <motion.button
                      key={`${category}-${index}`}
                      variants={animated ? itemVariants : {}}
                      initial="hidden"
                      animate="visible"
                      exit="hidden"
                      whileHover={{ scale: 1.05 }}
                      whileTap={{ scale: 0.95 }}
                      onClick={() => handleChipClick(chip)}
                      className={clsx(
                        'inline-flex items-center gap-1.5 rounded-full border transition-all duration-200 cursor-pointer focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500',
                        getChipColor(chip.type),
                        chipVariants[variant]
                      )}
                    >
                      <span>{getChipIcon(chip.type)}</span>
                      <span>{chip.text}</span>
                    </motion.button>
                  ))}
                </AnimatePresence>
              </div>
            </div>
          )
        ))}
      </motion.div>
    );
  }

  return (
    <motion.div
      initial="hidden"
      animate="visible"
      variants={containerVariants}
      className={clsx('flex flex-wrap gap-2', className)}
    >
      <AnimatePresence>
        {chips.map((chip, index) => (
          <motion.button
            key={index}
            variants={animated ? itemVariants : {}}
            initial="hidden"
            animate="visible"
            exit="hidden"
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
            onClick={() => handleChipClick(chip)}
            className={clsx(
              'inline-flex items-center gap-1.5 rounded-full border transition-all duration-200 cursor-pointer focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500',
              getChipColor(chip.type),
              chipVariants[variant]
            )}
          >
            <span>{getChipIcon(chip.type)}</span>
            <span>{chip.text}</span>
          </motion.button>
        ))}
      </AnimatePresence>
    </motion.div>
  );
};

// Hook for using keyword extraction
export const useKeywordExtraction = (text: string, options?: { maxChips?: number }) => {
  const [keywords, setKeywords] = useState<ChipData[]>([]);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (text) {
      setIsLoading(true);
      const extractor = new KeywordExtractor(options);

      setTimeout(() => {
        const extracted = extractor.extractKeywords(text);
        setKeywords(extracted);
        setIsLoading(false);
      }, 100);
    }
  }, [text, options?.maxChips]);

  return { keywords, isLoading };
};

// Standalone component for quick integration
export const QuickKeywordChips: React.FC<{ text: string; onChipClick?: (text: string) => void }> = ({
  text,
  onChipClick
}) => {
  const { keywords, isLoading } = useKeywordExtraction(text);

  if (isLoading) {
    return <div className="h-8 animate-pulse bg-gray-100 rounded-lg w-full" />;
  }

  return (
    <KeywordChips
      aiAnswer={text}
      onChipClick={onChipClick}
      maxChips={4}
      variant="compact"
    />
  );
};