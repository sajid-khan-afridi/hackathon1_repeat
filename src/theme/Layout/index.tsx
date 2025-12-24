/**
 * Swizzled Layout component to properly position ProfileBanner after the navbar.
 *
 * The ProfileBanner uses position: sticky with top: var(--ifm-navbar-height),
 * so it must be rendered AFTER the navbar in the DOM tree for proper positioning.
 *
 * Previously, ProfileBannerWrapper was in Root.tsx which rendered it BEFORE
 * the navbar, causing layout overlay issues.
 */

import React from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type { WrapperProps } from '@docusaurus/types';
import { ProfileBannerWrapper } from '@site/src/components/Profile/ProfileBannerWrapper';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <Layout {...props}>
        {/* Profile banner now renders inside Layout, after navbar */}
        <ProfileBannerWrapper />
        {props.children}
      </Layout>
    </>
  );
}
