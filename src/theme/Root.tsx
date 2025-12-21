import React, { ReactNode } from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import { UserProvider } from '@site/src/context/UserContext';
import { LanguageProvider } from '@site/src/context/LanguageContext';
import GlobalFloatingChat from '@site/src/components/GlobalFloatingChat';
import { ProfileBannerWrapper } from '@site/src/components/Profile/ProfileBannerWrapper';

interface RootProps {
  children: ReactNode;
}

/**
 * Root wrapper component for Docusaurus.
 * Docusaurus auto-discovers this file and uses it to wrap the entire app.
 * This enables global context providers for user profiles and language selection.
 * Also includes the GlobalFloatingChat widget available on all pages.
 */
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <AuthProvider>
      <UserProvider>
        <LanguageProvider>
        {/* Skip-to-content link for accessibility */}
        <a
          href="#__docusaurus"
          className="skip-link"
          style={{
            position: 'absolute',
            left: '-9999px',
            top: 'auto',
            width: '1px',
            height: '1px',
            overflow: 'hidden',
          }}
          onFocus={(e) => {
            e.currentTarget.style.position = 'fixed';
            e.currentTarget.style.left = '10px';
            e.currentTarget.style.top = '10px';
            e.currentTarget.style.width = 'auto';
            e.currentTarget.style.height = 'auto';
            e.currentTarget.style.overflow = 'visible';
            e.currentTarget.style.zIndex = '9999';
            e.currentTarget.style.padding = '8px 16px';
            e.currentTarget.style.backgroundColor = 'var(--ifm-color-primary)';
            e.currentTarget.style.color = 'white';
            e.currentTarget.style.borderRadius = '4px';
            e.currentTarget.style.textDecoration = 'none';
          }}
          onBlur={(e) => {
            e.currentTarget.style.position = 'absolute';
            e.currentTarget.style.left = '-9999px';
            e.currentTarget.style.width = '1px';
            e.currentTarget.style.height = '1px';
            e.currentTarget.style.overflow = 'hidden';
          }}
        >
          Skip to main content
        </a>

        {/* Profile completion banner - shows for authenticated users with incomplete profiles */}
        <ProfileBannerWrapper />

        {children}

        {/* Global floating chat widget - appears on all pages */}
        <GlobalFloatingChat />
        </LanguageProvider>
      </UserProvider>
    </AuthProvider>
  );
}
