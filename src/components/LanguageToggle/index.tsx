/**
 * LanguageToggle Component
 * Language switcher with ARIA support for English/Urdu
 * Phase 5: Translation Feature - T018
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import { useLanguageContext } from '../../context/LanguageContext';
import type { LanguageCode } from '../../types/translation';
import { LANGUAGE_CONFIGS } from '../../types/translation';
import styles from './LanguageToggle.module.css';

interface LanguageOption {
  code: LanguageCode;
  label: string;
  nativeLabel: string;
}

const languageOptions: LanguageOption[] = [
  { code: 'en', label: 'English', nativeLabel: 'English' },
  { code: 'ur', label: 'Urdu', nativeLabel: 'اردو' },
];

export interface LanguageToggleProps {
  /** Optional className for custom styling */
  className?: string;
  /** Show full label or just code */
  compact?: boolean;
}

export function LanguageToggle({ className, compact = false }: LanguageToggleProps): React.ReactElement {
  const { language, setLanguage, isLoading } = useLanguageContext();
  const [isOpen, setIsOpen] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);
  const menuRef = useRef<HTMLUListElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);

  const currentOption = languageOptions.find((opt) => opt.code === language) || languageOptions[0];

  // Close menu when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (containerRef.current && !containerRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Focus management for menu
  useEffect(() => {
    if (isOpen && menuRef.current) {
      const firstItem = menuRef.current.querySelector('[role="menuitem"]') as HTMLElement;
      firstItem?.focus();
    }
  }, [isOpen]);

  // Keyboard navigation
  const handleKeyDown = useCallback(
    (event: React.KeyboardEvent) => {
      switch (event.key) {
        case 'Escape':
          setIsOpen(false);
          buttonRef.current?.focus();
          break;
        case 'ArrowDown':
          event.preventDefault();
          if (!isOpen) {
            setIsOpen(true);
          } else if (menuRef.current) {
            const items = menuRef.current.querySelectorAll('[role="menuitem"]');
            const currentIndex = Array.from(items).findIndex(
              (item) => item === document.activeElement
            );
            const nextIndex = (currentIndex + 1) % items.length;
            (items[nextIndex] as HTMLElement).focus();
          }
          break;
        case 'ArrowUp':
          event.preventDefault();
          if (menuRef.current) {
            const items = menuRef.current.querySelectorAll('[role="menuitem"]');
            const currentIndex = Array.from(items).findIndex(
              (item) => item === document.activeElement
            );
            const prevIndex = currentIndex <= 0 ? items.length - 1 : currentIndex - 1;
            (items[prevIndex] as HTMLElement).focus();
          }
          break;
        case 'Enter':
        case ' ':
          if (!isOpen) {
            event.preventDefault();
            setIsOpen(true);
          }
          break;
        case 'Tab':
          if (isOpen) {
            setIsOpen(false);
          }
          break;
      }
    },
    [isOpen]
  );

  const handleToggle = useCallback(() => {
    setIsOpen((prev) => !prev);
  }, []);

  const handleSelect = useCallback(
    (code: LanguageCode) => {
      setLanguage(code);
      setIsOpen(false);
      buttonRef.current?.focus();
    },
    [setLanguage]
  );

  const handleMenuItemKeyDown = useCallback(
    (event: React.KeyboardEvent, code: LanguageCode) => {
      if (event.key === 'Enter' || event.key === ' ') {
        event.preventDefault();
        handleSelect(code);
      }
    },
    [handleSelect]
  );

  // Generate aria-label based on current language
  const getAriaLabel = () => {
    if (language === 'ur') {
      return 'زبان تبدیل کریں';
    }
    return 'Change language';
  };

  if (isLoading) {
    return (
      <div className={`${styles.languageToggle} ${styles.loading} ${className || ''}`}>
        <span className={styles.loadingIndicator} aria-hidden="true" />
      </div>
    );
  }

  return (
    <div
      ref={containerRef}
      className={`${styles.container} ${className || ''}`}
      onKeyDown={handleKeyDown}
    >
      <button
        ref={buttonRef}
        type="button"
        className={styles.languageToggle}
        onClick={handleToggle}
        aria-label={getAriaLabel()}
        aria-expanded={isOpen}
        aria-haspopup="menu"
        aria-controls="language-menu"
      >
        <span className={styles.icon} aria-hidden="true">
          <svg
            width="20"
            height="20"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <circle cx="12" cy="12" r="10" />
            <line x1="2" y1="12" x2="22" y2="12" />
            <path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
          </svg>
        </span>
        <span className={styles.label}>
          {compact ? currentOption.code.toUpperCase() : currentOption.nativeLabel}
        </span>
        <span className={styles.chevron} aria-hidden="true">
          <svg
            width="12"
            height="12"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            style={{ transform: isOpen ? 'rotate(180deg)' : 'none' }}
          >
            <polyline points="6 9 12 15 18 9" />
          </svg>
        </span>
      </button>

      {isOpen && (
        <ul
          ref={menuRef}
          id="language-menu"
          role="menu"
          className={styles.menu}
          aria-label={language === 'ur' ? 'دستیاب زبانیں' : 'Available languages'}
        >
          {languageOptions.map((option) => (
            <li key={option.code} role="presentation">
              <button
                type="button"
                role="menuitem"
                className={`${styles.menuItem} ${option.code === language ? styles.active : ''}`}
                onClick={() => handleSelect(option.code)}
                onKeyDown={(e) => handleMenuItemKeyDown(e, option.code)}
                aria-checked={option.code === language}
                tabIndex={0}
              >
                <span className={styles.menuItemLabel}>{option.nativeLabel}</span>
                {option.code !== option.nativeLabel && (
                  <span className={styles.menuItemSublabel}>{option.label}</span>
                )}
                {option.code === language && (
                  <span className={styles.checkmark} aria-hidden="true">
                    <svg
                      width="16"
                      height="16"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="3"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                    >
                      <polyline points="20 6 9 17 4 12" />
                    </svg>
                  </span>
                )}
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

export default LanguageToggle;
