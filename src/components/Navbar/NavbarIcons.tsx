/**
 * NavbarIcons Component
 *
 * Consistent icon system for navbar items using Lucide React.
 * Provides accessible icons with proper ARIA attributes.
 *
 * @see specs/007-enhance-ui/plan.md for design decisions
 */

import React from 'react';
import {
  BookOpen,
  MessageCircle,
  BookMarked,
  LogIn,
  Github,
  Home,
  Menu,
  X,
  ChevronDown,
  type LucideIcon,
} from 'lucide-react';

export type NavIconName =
  | 'modules'
  | 'start'
  | 'chat'
  | 'glossary'
  | 'login'
  | 'github'
  | 'home'
  | 'menu'
  | 'close'
  | 'dropdown';

export interface NavbarIconProps {
  /** Icon name to render */
  name: NavIconName;

  /** Size in pixels (default: 20) */
  size?: number;

  /** Additional CSS classes */
  className?: string;

  /** Accessible label (required for standalone icons) */
  'aria-label'?: string;

  /** Whether icon is decorative (hidden from screen readers) */
  decorative?: boolean;

  /** Inline styles */
  style?: React.CSSProperties;
}

const iconMap: Record<NavIconName, LucideIcon> = {
  modules: BookOpen,
  start: Home,
  chat: MessageCircle,
  glossary: BookMarked,
  login: LogIn,
  github: Github,
  home: Home,
  menu: Menu,
  close: X,
  dropdown: ChevronDown,
};

/**
 * NavbarIcons Component
 *
 * Renders consistent icons for navbar items.
 *
 * @example
 * ```tsx
 * // Icon with label
 * <NavbarIcon name="chat" aria-label="Open chat" />
 *
 * // Decorative icon (next to text)
 * <NavbarIcon name="modules" decorative />
 * ```
 */
export function NavbarIcon({
  name,
  size = 20,
  className,
  'aria-label': ariaLabel,
  decorative = false,
  style,
}: NavbarIconProps): React.ReactElement {
  const IconComponent = iconMap[name];

  if (!IconComponent) {
    console.warn(`NavbarIcon: Unknown icon name "${name}"`);
    return <span className={className} style={style} />;
  }

  return (
    <IconComponent
      size={size}
      className={className}
      style={style}
      aria-hidden={decorative}
      aria-label={decorative ? undefined : ariaLabel}
      role={decorative ? undefined : 'img'}
      focusable="false"
    />
  );
}

export default NavbarIcon;
