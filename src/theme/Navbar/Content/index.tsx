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
import { useAuth } from '../../../hooks/useAuth';

import styles from './styles.module.css';

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
  const loginUrl = useBaseUrl('/login');
  const profileUrl = useBaseUrl('/profile');
  const homeUrl = useBaseUrl('/');

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
        <a href={profileUrl} className={styles.profileLink} title="View Profile">
          👤 Profile
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
