// Type declarations for Docusaurus modules
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
