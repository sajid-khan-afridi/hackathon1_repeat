import React, { type ReactNode } from 'react';
import clsx from 'clsx';
import { useThemeConfig, ErrorCauseBoundary, ThemeClassNames } from '@docusaurus/theme-common';
import { splitNavbarItems, useNavbarMobileSidebar } from '@docusaurus/theme-common/internal';
import NavbarItem, { type Props as NavbarItemConfig } from '@theme/NavbarItem';
import NavbarColorModeToggle from '@theme/Navbar/ColorModeToggle';
import SearchBar from '@theme/SearchBar';
import NavbarMobileSidebarToggle from '@theme/Navbar/MobileSidebar/Toggle';
import NavbarLogo from '@theme/Navbar/Logo';
import NavbarSearch from '@theme/Navbar/Search';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useLocation } from '@docusaurus/router';
import { useAuth } from '../../../hooks/useAuth';
import { LanguageToggle } from '../../../components/LanguageToggle';

import styles from './styles.module.css';

/**
 * Profile icon SVG component for consistent rendering across browsers
 */
function ProfileIcon(): ReactNode {
  return (
    <svg className={styles.profileIcon} viewBox="0 0 24 24" aria-hidden="true">
      <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
    </svg>
  );
}

function useNavbarItems() {
  // TODO temporary casting until ThemeConfig type is improved
  return useThemeConfig().navbar.items as NavbarItemConfig[];
}

function NavbarItems({ items }: { items: NavbarItemConfig[] }): ReactNode {
  return (
    <>
      {items.map((item, i) => (
        <ErrorCauseBoundary
          key={i}
          onError={(error) =>
            new Error(
              `A theme navbar item failed to render.
Please double-check the following navbar item (themeConfig.navbar.items) of your Docusaurus config:
${JSON.stringify(item, null, 2)}`,
              { cause: error }
            )
          }
        >
          <NavbarItem {...item} />
        </ErrorCauseBoundary>
      ))}
    </>
  );
}

function NavbarContentLayout({ left, right }: { left: ReactNode; right: ReactNode }) {
  return (
    <div className="navbar__inner">
      <div className={clsx(ThemeClassNames.layout.navbar.containerLeft, 'navbar__items')}>
        {left}
      </div>
      <div
        className={clsx(
          ThemeClassNames.layout.navbar.containerRight,
          'navbar__items navbar__items--right'
        )}
      >
        {right}
      </div>
    </div>
  );
}

/**
 * Auth-aware navbar item that shows Login or User menu based on auth state.
 */
function AuthNavItem(): ReactNode {
  const { isAuthenticated, isLoading, user, logout } = useAuth();
  const location = useLocation();
  const loginUrl = useBaseUrl('/login');
  const profileUrl = useBaseUrl('/profile');
  const homeUrl = useBaseUrl('/');

  // Check if currently on profile page for active state
  const isOnProfilePage = location.pathname.includes('/profile');

  const handleLogout = async () => {
    await logout();
    window.location.href = homeUrl;
  };

  // Show nothing while loading to prevent flash
  if (isLoading) {
    return null;
  }

  if (isAuthenticated && user) {
    return (
      <div className={styles.authNavItem}>
        <a
          href={profileUrl}
          className={clsx(
            styles.profileLink,
            isOnProfilePage && styles.profileLinkActive
          )}
          title="View Profile"
          aria-current={isOnProfilePage ? 'page' : undefined}
        >
          <ProfileIcon />
          Profile
        </a>
        <span className={styles.userEmail}>{user.email}</span>
        <button
          className={clsx('navbar__item', styles.logoutButton)}
          onClick={handleLogout}
          type="button"
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <a href={loginUrl} className="navbar__item navbar__link">
      Login
    </a>
  );
}

export default function NavbarContent(): ReactNode {
  const mobileSidebar = useNavbarMobileSidebar();

  const items = useNavbarItems();
  const [leftItems, rightItems] = splitNavbarItems(items);

  // Filter out the static "Login" item - we'll render AuthNavItem instead
  const filteredRightItems = rightItems.filter(
    (item) => !('to' in item && item.to === '/login')
  );

  const searchBarItem = items.find((item) => item.type === 'search');

  return (
    <NavbarContentLayout
      left={
        // TODO stop hardcoding items?
        <>
          {!mobileSidebar.disabled && <NavbarMobileSidebarToggle />}
          <NavbarLogo />
          <NavbarItems items={leftItems} />
        </>
      }
      right={
        // TODO stop hardcoding items?
        // Ask the user to add the respective navbar items => more flexible
        <>
          <AuthNavItem />
          <NavbarItems items={filteredRightItems} />
          <LanguageToggle className={styles.languageToggle} />
          <NavbarColorModeToggle className={styles.colorModeToggle} />
          {!searchBarItem && (
            <NavbarSearch>
              <SearchBar />
            </NavbarSearch>
          )}
        </>
      }
    />
  );
}
