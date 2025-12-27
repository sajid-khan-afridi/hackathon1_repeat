/**
 * MobileDrawer Component
 *
 * Slide-out navigation drawer for mobile devices.
 * Supports keyboard navigation and focus trapping.
 *
 * @see specs/007-enhance-ui/plan.md for design decisions
 */

import React, { useEffect, useRef, useCallback } from 'react';
import clsx from 'clsx';
import { NavbarIcon } from './NavbarIcons';
import styles from './MobileDrawer.module.css';

export interface DrawerItem {
  /** Unique identifier */
  id: string;

  /** Display label */
  label: string;

  /** Navigation URL */
  href?: string;

  /** Click handler (alternative to href) */
  onClick?: () => void;

  /** Icon name from NavbarIcons */
  icon?: 'modules' | 'start' | 'chat' | 'glossary' | 'login' | 'github' | 'home';

  /** Whether item is currently active */
  isActive?: boolean;

  /** Sub-items for nested navigation */
  items?: DrawerItem[];
}

export interface MobileDrawerProps {
  /** Whether drawer is open */
  isOpen: boolean;

  /** Callback when drawer should close */
  onClose: () => void;

  /** Navigation items */
  items: DrawerItem[];

  /** Site title */
  title?: string;

  /** Logo element */
  logo?: React.ReactNode;

  /** Additional CSS classes */
  className?: string;
}

/**
 * MobileDrawer Component
 *
 * @example
 * ```tsx
 * <MobileDrawer
 *   isOpen={isMenuOpen}
 *   onClose={() => setIsMenuOpen(false)}
 *   items={navItems}
 *   title="Robotics Textbook"
 * />
 * ```
 */
export function MobileDrawer({
  isOpen,
  onClose,
  items,
  title,
  logo,
  className,
}: MobileDrawerProps): React.ReactElement | null {
  const drawerRef = useRef<HTMLDivElement>(null);
  const closeButtonRef = useRef<HTMLButtonElement>(null);
  const previousFocusRef = useRef<HTMLElement | null>(null);

  // Focus management
  useEffect(() => {
    if (isOpen) {
      previousFocusRef.current = document.activeElement as HTMLElement;
      // Focus close button after animation
      setTimeout(() => {
        closeButtonRef.current?.focus();
      }, 100);
    } else if (previousFocusRef.current) {
      previousFocusRef.current.focus();
      previousFocusRef.current = null;
    }
  }, [isOpen]);

  // Keyboard navigation
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
        return;
      }

      // Focus trap within drawer
      if (e.key === 'Tab' && drawerRef.current) {
        const focusableElements = drawerRef.current.querySelectorAll<HTMLElement>(
          'button, a[href], input, [tabindex]:not([tabindex="-1"])'
        );

        if (focusableElements.length === 0) return;

        const first = focusableElements[0];
        const last = focusableElements[focusableElements.length - 1];

        if (e.shiftKey && document.activeElement === first) {
          e.preventDefault();
          last.focus();
        } else if (!e.shiftKey && document.activeElement === last) {
          e.preventDefault();
          first.focus();
        }
      }
    },
    [onClose]
  );

  // Prevent body scroll when open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen]);

  // Render drawer item
  const renderItem = (item: DrawerItem, depth = 0) => {
    const ItemTag = item.href ? 'a' : 'button';
    const itemProps = item.href
      ? { href: item.href }
      : { type: 'button' as const, onClick: item.onClick };

    return (
      <li key={item.id} className={styles.navItem}>
        <ItemTag
          {...itemProps}
          className={clsx(styles.navLink, {
            [styles.active]: item.isActive,
            [styles.nested]: depth > 0,
          })}
          onClick={(e) => {
            item.onClick?.();
            if (item.href) {
              onClose();
            }
          }}
          style={{ paddingLeft: `${1 + depth * 0.75}rem` }}
        >
          {item.icon && (
            <NavbarIcon
              name={item.icon}
              size={20}
              decorative
              className={styles.navIcon}
            />
          )}
          <span>{item.label}</span>
        </ItemTag>
        {item.items && item.items.length > 0 && (
          <ul className={styles.subNav}>
            {item.items.map((subItem) => renderItem(subItem, depth + 1))}
          </ul>
        )}
      </li>
    );
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div
      className={styles.overlay}
      onClick={onClose}
      aria-hidden="true"
    >
      <div
        ref={drawerRef}
        className={clsx(styles.drawer, className)}
        role="dialog"
        aria-modal="true"
        aria-label="Navigation menu"
        onClick={(e) => e.stopPropagation()}
        onKeyDown={handleKeyDown}
      >
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.brand}>
            {logo}
            {title && <span className={styles.title}>{title}</span>}
          </div>
          <button
            ref={closeButtonRef}
            type="button"
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close navigation menu"
          >
            <NavbarIcon name="close" size={24} decorative />
          </button>
        </div>

        {/* Navigation */}
        <nav className={styles.nav}>
          <ul className={styles.navList}>
            {items.map((item) => renderItem(item))}
          </ul>
        </nav>
      </div>
    </div>
  );
}

export default MobileDrawer;
