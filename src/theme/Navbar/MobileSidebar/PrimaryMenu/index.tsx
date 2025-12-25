import React, { type ReactNode } from 'react';
import { useThemeConfig } from '@docusaurus/theme-common';
import { useNavbarMobileSidebar } from '@docusaurus/theme-common/internal';
import NavbarItem, { type Props as NavbarItemConfig } from '@theme/NavbarItem';
import useBaseUrl from '@docusaurus/useBaseUrl';
import { useAuth } from '../../../../hooks/useAuth';

import styles from './styles.module.css';

function useNavbarItems() {
  // TODO temporary casting until ThemeConfig type is improved
  return useThemeConfig().navbar.items as NavbarItemConfig[];
}

/**
 * Auth section for mobile sidebar - shows Profile/Logout when authenticated
 */
function MobileAuthSection(): ReactNode {
  const { isAuthenticated, isLoading, user, logout } = useAuth();
  const mobileSidebar = useNavbarMobileSidebar();
  const loginUrl = useBaseUrl('/login');
  const profileUrl = useBaseUrl('/profile');
  const homeUrl = useBaseUrl('/');

  const handleLogout = async () => {
    mobileSidebar.toggle();
    await logout();
    window.location.href = homeUrl;
  };

  const handleProfileClick = () => {
    mobileSidebar.toggle();
  };

  if (isLoading) {
    return null;
  }

  if (isAuthenticated && user) {
    return (
      <div className={styles.mobileAuthSection}>
        <div className={styles.mobileUserInfo}>
          <svg className={styles.mobileUserIcon} viewBox="0 0 24 24" aria-hidden="true">
            <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
          </svg>
          <span className={styles.mobileUserEmail}>{user.email}</span>
        </div>
        <a
          href={profileUrl}
          className={styles.mobileProfileLink}
          onClick={handleProfileClick}
        >
          Profile Settings
        </a>
        <button
          className={styles.mobileLogoutButton}
          onClick={handleLogout}
          type="button"
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <div className={styles.mobileAuthSection}>
      <a
        href={loginUrl}
        className={styles.mobileLoginLink}
        onClick={() => mobileSidebar.toggle()}
      >
        Login
      </a>
    </div>
  );
}

// The primary menu displays the navbar items
export default function NavbarMobilePrimaryMenu(): ReactNode {
  const mobileSidebar = useNavbarMobileSidebar();

  // TODO how can the order be defined for mobile?
  // Should we allow providing a different list of items?
  const items = useNavbarItems();

  // Filter out Login item - MobileAuthSection handles auth links to avoid duplicates
  const filteredItems = items.filter((item) => {
    // Filter out items that link to login page
    if ('to' in item && item.to === '/login') return false;
    if ('href' in item && typeof item.href === 'string' && item.href.includes('/login')) return false;
    // Filter out items with label "Login"
    if ('label' in item && item.label === 'Login') return false;
    return true;
  });

  return (
    <>
      <ul className="menu__list">
        {filteredItems.map((item, i) => (
          <NavbarItem mobile {...item} onClick={() => mobileSidebar.toggle()} key={i} />
        ))}
      </ul>
      <MobileAuthSection />
    </>
  );
}
