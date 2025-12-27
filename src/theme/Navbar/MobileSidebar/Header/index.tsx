import React, { type ReactNode, useEffect, useRef, useCallback } from 'react';
import { useNavbarMobileSidebar } from '@docusaurus/theme-common/internal';
import { translate } from '@docusaurus/Translate';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import IconClose from '@theme/Icon/Close';
import NavbarLogo from '@theme/Navbar/Logo';

function CloseButton() {
  const mobileSidebar = useNavbarMobileSidebar();
  const buttonRef = useRef<HTMLButtonElement>(null);

  // Focus close button when sidebar opens for keyboard accessibility
  useEffect(() => {
    if (mobileSidebar.shown && buttonRef.current) {
      // Small delay to ensure animation has started
      const timeoutId = setTimeout(() => {
        buttonRef.current?.focus();
      }, 100);
      return () => clearTimeout(timeoutId);
    }
  }, [mobileSidebar.shown]);

  // Handle Escape key to close sidebar
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === 'Escape') {
        mobileSidebar.toggle();
      }
    },
    [mobileSidebar]
  );

  return (
    <button
      ref={buttonRef}
      type="button"
      aria-label={translate({
        id: 'theme.docs.sidebar.closeSidebarButtonAriaLabel',
        message: 'Close navigation bar',
        description: 'The ARIA label for close button of mobile sidebar',
      })}
      className="clean-btn navbar-sidebar__close"
      onClick={() => mobileSidebar.toggle()}
      onKeyDown={handleKeyDown}
    >
      <IconClose color="var(--ifm-color-emphasis-600)" />
    </button>
  );
}

export default function NavbarMobileSidebarHeader(): ReactNode {
  return (
    <div className="navbar-sidebar__brand">
      <NavbarLogo />
      <NavbarColorModeToggle className="margin-right--md" />
      <CloseButton />
    </div>
  );
}
