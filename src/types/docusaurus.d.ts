// Type declarations for Docusaurus modules
import React from 'react';

// Ensure JSX namespace is available globally
declare global {
  namespace JSX {
    interface IntrinsicElements {
      [elemName: string]: any;
    }
    type Element = React.ReactElement<any, any>;
  }
}

declare module '@theme/Heading' {
  import { ComponentType } from 'react';
  interface HeadingProps {
    as?: 'h1' | 'h2' | 'h3' | 'h4' | 'h5' | 'h6';
    id?: string;
    children: React.ReactNode;
  }
  const Heading: ComponentType<HeadingProps>;
  export default Heading;
}

declare module '@docusaurus/Link' {
  import { ComponentType } from 'react';
  interface LinkProps {
    to: string;
    children: React.ReactNode;
    [key: string]: any;
  }
  const Link: ComponentType<LinkProps>;
  export default Link;
}

declare module '@theme/Layout' {
  import { ComponentType } from 'react';
  interface LayoutProps {
    children: React.ReactNode;
    title?: string;
    description?: string;
  }
  const Layout: ComponentType<LayoutProps>;
  export default Layout;
}

declare module '@theme/Logo' {
  import { ComponentType } from 'react';
  interface LogoProps {
    className?: string;
    [key: string]: any;
  }
  const Logo: ComponentType<LogoProps>;
  export default Logo;
}

declare module '@theme/NavbarItem' {
  import { ComponentType } from 'react';
  interface NavbarItemProps {
    [key: string]: any;
  }
  const NavbarItem: ComponentType<NavbarItemProps>;
  export default NavbarItem;
}

declare module '@theme/SearchBar' {
  import { ComponentType } from 'react';
  interface SearchBarProps {
    [key: string]: any;
  }
  const SearchBar: ComponentType<SearchBarProps>;
  export default SearchBar;
}

declare module '@theme/ColorModeToggle' {
  import { ComponentType } from 'react';
  interface ColorModeToggleProps {
    [key: string]: any;
  }
  const ColorModeToggle: ComponentType<ColorModeToggleProps>;
  export default ColorModeToggle;
}

declare module '@theme/Navbar/ColorModeToggle' {
  import { ComponentType } from 'react';
  interface ColorModeToggleProps {
    [key: string]: any;
  }
  const ColorModeToggle: ComponentType<ColorModeToggleProps>;
  export default ColorModeToggle;
}

declare module '@theme/Navbar/MobileSidebar/Toggle' {
  import { ComponentType } from 'react';
  interface ToggleProps {
    [key: string]: any;
  }
  const Toggle: ComponentType<ToggleProps>;
  export default Toggle;
}

declare module '@theme/Navbar/Logo' {
  import { ComponentType } from 'react';
  interface LogoProps {
    [key: string]: any;
  }
  const Logo: ComponentType<LogoProps>;
  export default Logo;
}

declare module '@theme/Navbar/Search' {
  import { ComponentType } from 'react';
  interface SearchProps {
    [key: string]: any;
  }
  const Search: ComponentType<SearchProps>;
  export default Search;
}

declare module '@theme/Navbar/MobileSidebar' {
  import { ComponentType } from 'react';
  interface MobileSidebarProps {
    [key: string]: any;
  }
  const MobileSidebar: ComponentType<MobileSidebarProps>;
  export default MobileSidebar;
}

declare module '@theme/Navbar/MobileSidebar/Layout' {
  import { ComponentType } from 'react';
  interface LayoutProps {
    [key: string]: any;
  }
  const Layout: ComponentType<LayoutProps>;
  export default Layout;
}

declare module '@theme/Navbar/MobileSidebar/Header' {
  import { ComponentType } from 'react';
  interface HeaderProps {
    [key: string]: any;
  }
  const Header: ComponentType<HeaderProps>;
  export default Header;
}

declare module '@theme/Navbar/MobileSidebar/PrimaryMenu' {
  import { ComponentType } from 'react';
  interface PrimaryMenuProps {
    [key: string]: any;
  }
  const PrimaryMenu: ComponentType<PrimaryMenuProps>;
  export default PrimaryMenu;
}

declare module '@theme/Navbar/MobileSidebar/SecondaryMenu' {
  import { ComponentType } from 'react';
  interface SecondaryMenuProps {
    [key: string]: any;
  }
  const SecondaryMenu: ComponentType<SecondaryMenuProps>;
  export default SecondaryMenu;
}

declare module '@theme/Navbar/Layout' {
  import { ComponentType } from 'react';
  interface LayoutProps {
    [key: string]: any;
  }
  const Layout: ComponentType<LayoutProps>;
  export default Layout;
}

declare module '@theme/Icon/Close' {
  import { ComponentType } from 'react';
  interface CloseIconProps {
    [key: string]: any;
  }
  const CloseIcon: ComponentType<CloseIconProps>;
  export default CloseIcon;
}

declare module '@theme/Icon/Menu' {
  import { ComponentType } from 'react';
  interface MenuIconProps {
    [key: string]: any;
  }
  const MenuIcon: ComponentType<MenuIconProps>;
  export default MenuIcon;
}

declare module '@theme/Navbar/Search' {
  import { ComponentType } from 'react';
  interface SearchProps {
    [key: string]: any;
  }
  const Search: ComponentType<SearchProps>;
  export default Search;
}

declare module '@docusaurus/useDocusaurusContext' {
  export function useDocusaurusContext(): {
    siteConfig: {
      title: string;
      tagline: string;
      url: string;
      baseUrl: string;
      [key: string]: any;
    };
    i18n: {
      currentLocale: string;
      localeConfigs: any;
    };
  };
}

declare module '@docusaurus/Translate' {
  import { ComponentType } from 'react';
  interface TranslateProps {
    id: string;
    description?: string;
    children?: React.ReactNode;
    [key: string]: any;
  }
  export const translate: (id: string, opts?: any) => string;
  export const Translate: ComponentType<TranslateProps>;
}

declare module '@site/src/context/UserContext' {
  import { Context } from 'react';
  export interface UserContextType {
    user: {
      id: string;
      name: string;
      preferences: {
        skillLevel: 'beginner' | 'intermediate' | 'advanced';
        learningGoals: string[];
      };
    };
    updateUser: (user: any) => void;
  }
  export const UserContext: Context<UserContextType>;
  export const UserProvider: ComponentType<{ children: React.ReactNode }>;
}

declare module '@site/src/context/LanguageContext' {
  import { Context } from 'react';
  export interface LanguageContextType {
    language: string;
    setLanguage: (lang: string) => void;
    t: (key: string) => string;
  }
  export const LanguageContext: Context<LanguageContextType>;
  export const LanguageProvider: ComponentType<{ children: React.ReactNode }>;
}

declare module '@site/src/data/technical-terms.json' {
  const terms: Record<
    string,
    {
      term: string;
      definition: string;
      category: string;
      relatedTerms?: string[];
    }
  >;
  export default terms;
}

declare module '@site/src/components/HomepageFeatures' {
  import { ComponentType } from 'react';
  interface HomepageFeaturesProps {
    [key: string]: any;
  }
  const HomepageFeatures: ComponentType<HomepageFeaturesProps>;
  export default HomepageFeatures;
}

// Import React for JSX namespace
import 'react';

// Additional exports for theme-common
declare module '@docusaurus/theme-common' {
  export function useThemeConfig(): any;
  export function useColorMode(): {
    colorMode: 'light' | 'dark';
    setColorMode: (mode: 'light' | 'dark') => void;
  };
  export function useNavbarSecondaryMenu(): {
    shown: boolean;
    toggle: () => void;
  };
  export const ThemeClassNames: {
    layout: {
      navbar: {
        mobileSidebar: {
          container: string;
          panel: string;
        };
      };
    };
  };
  export function ErrorCauseBoundary(props: any): React.JSX.Element;
}

declare module '@docusaurus/theme-common/internal' {
  export function useNavbarSecondaryMenu(): {
    shown: boolean;
    toggle: () => void;
  };
}

// Type for Props interface
declare module '@theme/Navbar/ColorModeToggle' {
  export interface Props {
    className?: string;
  }
}

declare module '@theme/Navbar/Layout' {
  export interface Props {
    children: React.ReactNode;
  }
}

declare module '@theme/Navbar/MobileSidebar/Layout' {
  export interface Props {
    header: React.ReactNode;
    primaryMenu: React.ReactNode;
    secondaryMenu: React.ReactNode;
  }
}

// Missing module declarations for @docusaurus/*
declare module '@docusaurus/useBaseUrl' {
  export default function useBaseUrl(url: string, options?: any): string;
}

declare module '@docusaurus/router' {
  export function useLocation(): {
    pathname: string;
    search: string;
    hash: string;
    state: any;
    key: string;
  };
  export function useHistory(): {
    push: (path: string, state?: any) => void;
    replace: (path: string, state?: any) => void;
    go: (n: number) => void;
    goBack: () => void;
    goForward: () => void;
  };
  export const Link: React.ComponentType<{
    to: string;
    children: React.ReactNode;
    [key: string]: any;
  }>;
  export const NavLink: React.ComponentType<{
    to: string;
    children: React.ReactNode;
    [key: string]: any;
  }>;
  export const Redirect: React.ComponentType<{
    to: string;
  }>;
}

// Missing @theme/* declarations
declare module '@theme/Icon/LightMode' {
  import { ComponentType } from 'react';
  const LightModeIcon: ComponentType<{ [key: string]: any }>;
  export default LightModeIcon;
}

declare module '@theme/Icon/DarkMode' {
  import { ComponentType } from 'react';
  const DarkModeIcon: ComponentType<{ [key: string]: any }>;
  export default DarkModeIcon;
}

declare module '@theme/Icon/SystemColorMode' {
  import { ComponentType } from 'react';
  const SystemColorModeIcon: ComponentType<{ [key: string]: any }>;
  export default SystemColorModeIcon;
}

// @theme-original/* declarations for swizzled components
declare module '@theme-original/Layout' {
  import { ComponentType } from 'react';
  const Layout: ComponentType<{ children: React.ReactNode; [key: string]: any }>;
  export default Layout;
}

declare module '@theme-original/DocItem/Content' {
  import { ComponentType } from 'react';
  const Content: ComponentType<{ [key: string]: any }>;
  export default Content;
}

declare module '@theme/DocItem/Content' {
  import { ComponentType } from 'react';
  export interface Props {
    [key: string]: any;
  }
  const Content: ComponentType<Props>;
  export default Content;
}

// @site/* module declarations
declare module '@site/src/context/AuthContext' {
  import { Context, ComponentType } from 'react';
  export interface User {
    id: string;
    email: string;
    name: string;
    [key: string]: any;
  }
  export interface AuthContextType {
    isAuthenticated: boolean;
    user: User | null;
    login: (email: string, password: string) => Promise<void>;
    logout: () => Promise<void>;
    [key: string]: any;
  }
  export const AuthContext: Context<AuthContextType>;
  export const AuthProvider: ComponentType<{ children: React.ReactNode }>;
  export function useAuth(): AuthContextType;
}

declare module '@site/src/hooks/useAuth' {
  export interface User {
    id: string;
    email: string;
    name: string;
    [key: string]: any;
  }
  export function useAuth(): {
    isAuthenticated: boolean;
    user: User | null;
    login: (email: string, password: string) => Promise<void>;
    logout: () => Promise<void>;
    [key: string]: any;
  };
}

declare module '@site/src/hooks/useChapterProgress' {
  export interface ChapterProgress {
    chapterId: string;
    progress: number;
    completed: boolean;
    [key: string]: any;
  }
  export function useChapterProgress(chapterId?: string): {
    progress: ChapterProgress | null;
    updateProgress: (progress: number) => void;
    markComplete: () => void;
    [key: string]: any;
  };
}

declare module '@site/src/components/ChatbotWidget' {
  import { ComponentType } from 'react';
  const ChatbotWidget: ComponentType<{ [key: string]: any }>;
  export default ChatbotWidget;
}

declare module '@site/src/components/GlossaryPage' {
  import { ComponentType } from 'react';
  const GlossaryPage: ComponentType<{ [key: string]: any }>;
  export default GlossaryPage;
}

declare module '@site/src/components/ProgressTracker' {
  import { ComponentType } from 'react';
  export interface ProgressTrackerProps {
    chapterId?: string;
    [key: string]: any;
  }
  const ProgressTracker: ComponentType<ProgressTrackerProps>;
  export default ProgressTracker;
}

declare module '@site/src/components/PersonalizedSection' {
  import { ComponentType } from 'react';
  const PersonalizedSection: ComponentType<{ children: React.ReactNode; [key: string]: any }>;
  export default PersonalizedSection;
}

declare module '@site/src/components/TechnicalTerm' {
  import { ComponentType } from 'react';
  const TechnicalTerm: ComponentType<{ term: string; [key: string]: any }>;
  export default TechnicalTerm;
}

declare module '@site/src/components/GlossaryTooltip' {
  import { ComponentType } from 'react';
  const GlossaryTooltip: ComponentType<{ term: string; children?: React.ReactNode; [key: string]: any }>;
  export default GlossaryTooltip;
}

declare module '@site/src/components/Profile/ProfileBannerWrapper' {
  import { ComponentType } from 'react';
  const ProfileBannerWrapper: ComponentType<{ [key: string]: any }>;
  export default ProfileBannerWrapper;
}

declare module '@site/src/components/animations' {
  import { ComponentType } from 'react';
  export const FadeIn: ComponentType<{ children: React.ReactNode; [key: string]: any }>;
  export const SlideIn: ComponentType<{ children: React.ReactNode; [key: string]: any }>;
  export const CountUp: ComponentType<{ end: number; [key: string]: any }>;
}

declare module '@site/src/types/auth' {
  export interface User {
    id: string;
    email: string;
    name: string;
    [key: string]: any;
  }
  export interface LoginRequest {
    email: string;
    password: string;
  }
  export interface SignupRequest {
    email: string;
    password: string;
    name: string;
  }
}

// Add ColorMode export to theme-common
declare module '@docusaurus/theme-common' {
  export type ColorMode = 'light' | 'dark';
  export function useThemeConfig(): any;
  export function useColorMode(): {
    colorMode: ColorMode;
    setColorMode: (mode: ColorMode) => void;
    isDarkTheme: boolean;
  };
  export function useNavbarSecondaryMenu(): {
    shown: boolean;
    toggle: () => void;
  };
  export const ThemeClassNames: {
    layout: {
      navbar: {
        containerLeft: string;
        containerRight: string;
        mobileSidebar: {
          container: string;
          panel: string;
        };
      };
    };
  };
  export function ErrorCauseBoundary(props: { onError: (error: Error) => Error; children: React.ReactNode }): React.JSX.Element;
}

declare module '@docusaurus/theme-common/internal' {
  export function useNavbarSecondaryMenu(): {
    shown: boolean;
    toggle: () => void;
  };
  export function useNavbarMobileSidebar(): {
    disabled: boolean;
    shouldRender: boolean;
    toggle: () => void;
    shown: boolean;
  };
  export function splitNavbarItems<T>(items: T[]): [T[], T[]];
}
