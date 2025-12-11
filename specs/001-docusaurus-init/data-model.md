# Data Model: Book Infrastructure

**Date**: 2025-12-10
**Feature**: 001-docusaurus-init
**Phase**: 1 - Design

## Theme Configuration Schema

### Color Palette

```typescript
interface ThemeColors {
  // Primary colors
  primary: string;
  primaryHover: string;
  primaryActive: string;

  // Secondary colors
  secondary: string;
  secondaryHover: string;
  accent: string;

  // Text colors
  textPrimary: string;
  textSecondary: string;
  textMuted: string;
  textLink: string;

  // Background colors
  backgroundPrimary: string;
  backgroundSecondary: string;
  backgroundTertiary: string;

  // Border colors
  borderPrimary: string;
  borderSecondary: string;
  borderFocus: string;

  // Status colors
  success: string;
  warning: string;
  error: string;
  info: string;
}
```

### Typography Scale

```typescript
interface TypographyScale {
  // Font families
  fontFamilyPrimary: string;
  fontFamilyMono: string;

  // Font sizes (rem units)
  fontSizeXs: string;    // 0.75rem
  fontSizeSm: string;    // 0.875rem
  fontSizeBase: string;  // 1rem
  fontSizeLg: string;    // 1.125rem
  fontSizeXl: string;    // 1.25rem
  fontSize2xl: string;   // 1.5rem
  fontSize3xl: string;   // 1.875rem
  fontSize4xl: string;   // 2.25rem

  // Font weights
  fontWeightNormal: number;
  fontWeightMedium: number;
  fontWeightSemibold: number;
  fontWeightBold: number;

  // Line heights
  lineHeightTight: number;
  lineHeightNormal: number;
  lineHeightRelaxed: number;
}
```

### Spacing Scale

```typescript
interface SpacingScale {
  // Base unit: 0.25rem (4px)
  space1: string;   // 0.25rem
  space2: string;   // 0.5rem
  space3: string;   // 0.75rem
  space4: string;   // 1rem
  space5: string;   // 1.25rem
  space6: string;   // 1.5rem
  space8: string;   // 2rem
  space10: string;  // 2.5rem
  space12: string;  // 3rem
  space16: string;  // 4rem
  space20: string;  // 5rem
  space24: string;  // 6rem
}
```

### Breakpoint Configuration

```typescript
interface Breakpoints {
  mobile: {
    max: '767px';
    columns: 1;
    gutter: '1rem';
  };
  tablet: {
    min: '768px';
    max: '1023px';
    columns: 2;
    gutter: '1.5rem';
  };
  desktop: {
    min: '1024px';
    columns: 3;
    gutter: '2rem';
  };
  wide: {
    min: '1440px';
    columns: 4;
    gutter: '2rem';
  };
}
```

## Navigation Structure

### Site Navigation

```typescript
interface NavigationItem {
  type: 'doc' | 'category' | 'link' | 'dropdown';

  // Document link
  id?: string;

  // Category
  label?: string;
  items?: NavigationItem[];

  // External link
  href?: string;
  external?: boolean;

  // Common
  customProps?: Record<string, any>;
}
```

### Sidebar Configuration

```typescript
interface SidebarConfig {
  // Main navigation sidebar
  mainSidebar: {
    type: 'category';
    label: 'Main Navigation';
    items: [
      // Getting Started
      {
        type: 'category';
        label: 'Getting Started';
        items: [
          { type: 'doc', id: 'intro' },
          { type: 'doc', id: 'installation' },
          { type: 'doc', id: 'quickstart' }
        ];
      },
      // Core Concepts
      {
        type: 'category';
        label: 'Core Concepts';
        items: [
          { type: 'doc', id: 'physical-ai/overview' },
          { type: 'doc', id: 'humanoid-robotics/intro' },
          { type: 'doc', id: 'ros2/fundamentals' }
        ];
      },
      // Tutorials
      {
        type: 'category';
        label: 'Tutorials';
        items: [
          { type: 'doc', id: 'tutorials/first-robot' },
          { type: 'doc', id: 'tutorials/ros2-basics' },
          { type: 'doc', id: 'tutorials/simulation' }
        ];
      }
    ];
  };

  // Secondary navigation (optional)
  tutorialSidebar?: SidebarConfig;
  apiSidebar?: SidebarConfig;
}
```

## Theme Toggle Component Schema

### Component Props

```typescript
interface ThemeToggleProps {
  // Display options
  showLabel?: boolean;
  label?: string;

  // Icon options
  icons?: {
    light: React.ReactNode;
    dark: React.ReactNode;
  };

  // Size variants
  size?: 'sm' | 'md' | 'lg';

  // Custom styles
  className?: string;
  style?: React.CSSProperties;

  // Accessibility
  'aria-label'?: string;
  'aria-labelledby'?: string;
}
```

### Theme Context Interface

```typescript
interface ThemeContextType {
  // Current theme state
  isDarkTheme: boolean;
  colorMode: 'light' | 'dark' | 'system';

  // Theme actions
  setColorMode: (mode: 'light' | 'dark' | 'system') => void;
  toggleColorMode: () => void;

  // System preference
  prefersDark: boolean;

  // Theme configuration
  themeConfig: ThemeConfig;
}
```

## Page Structure Schema

### Document Front Matter

```typescript
interface DocumentFrontMatter {
  // Basic metadata
  title: string;
  description?: string;

  // Navigation
  sidebar_label?: string;
  sidebar_position?: number;

  // Display options
  hide_table_of_contents?: boolean;
  pagination_next?: string;
  pagination_prev?: string;

  // Custom metadata
  tags?: string[];
  authors?: string[];
  last_updated?: string;
  difficulty?: 'beginner' | 'intermediate' | 'advanced';

  // Learning objectives
  learning_objectives?: string[];

  // Prerequisites
  prerequisites?: string[];

  // Code examples
  code_language?: string;
  runnable_examples?: boolean;
}
```

### TOC Configuration

```typescript
interface TableOfContents {
  minHeadingLevel: number; // Default: 2
  maxHeadingLevel: number; // Default: 3
  className?: string;

  // Custom styling
  style?: React.CSSProperties;

  // Headings to exclude
  excludeHeadings?: string[];
}
```

## Responsive Layout Schema

### Container Constraints

```typescript
interface ContainerConfig {
  // Maximum widths
  maxWidth: {
    xs: '100%';
    sm: '100%';
    md: '768px';
    lg: '1024px';
    xl: '1280px';
    '2xl': '1536px';
  };

  // Padding
  padding: {
    xs: '1rem';
    sm: '1.5rem';
    md: '2rem';
    lg: '2rem';
    xl: '3rem';
  };

  // Grid configuration
  grid: {
    columns: 12;
    gutter: '1rem';
  };
}
```

### Component Breakpoints

```typescript
interface ResponsiveProps<T> {
  xs?: T;
  sm?: T;
  md?: T;
  lg?: T;
  xl?: T;
  '2xl'?: T;
}

// Example usage
interface ButtonProps {
  size: ResponsiveProps<'sm' | 'md' | 'lg'>;
  fullWidth: ResponsiveProps<boolean>;
}
```

## Performance Monitoring Schema

### Performance Metrics

```typescript
interface PerformanceMetrics {
  // Core Web Vitals
  lcp: number;  // Largest Contentful Paint (target: <2.5s)
  fid: number;  // First Input Delay (target: <100ms)
  cls: number;  // Cumulative Layout Shift (target: <0.1)

  // Other metrics
  fcp: number;  // First Contentful Paint (target: <1.8s)
  ttfb: number; // Time to First Byte (target: <800ms)

  // Custom metrics
  themeToggleTime: number; // Time to switch theme (target: <100ms)
  navigationTime: number;  // Time to navigate (target: <200ms)
}
```

## Build Configuration Schema

### Docusaurus Configuration

```typescript
interface DocusaurusConfig {
  // Site metadata
  title: string;
  tagline: string;
  url: string;
  baseUrl: string;

  // Theme configuration
  themeConfig: {
    colorMode: {
      defaultMode: 'system' | 'light' | 'dark';
      respectPrefersColorScheme: boolean;
      disableSwitch: boolean;
    };
    navbar: NavbarConfig;
    footer: FooterConfig;
  };

  // Plugins
  plugins: PluginConfig[];

  // Custom fields
  customFields: {
    repositoryUrl: string;
    contributors: string[];
    lastUpdated: string;
  };
}
```

## Validation Rules

### Theme Validation

```typescript
interface ValidationRule {
  field: string;
  required: boolean;
  type: 'string' | 'number' | 'boolean' | 'array' | 'object';
  pattern?: RegExp;
  min?: number;
  max?: number;
  enum?: any[];
  custom?: (value: any) => boolean | string;
}

const themeValidationRules: ValidationRule[] = [
  {
    field: 'colors.primary',
    required: true,
    type: 'string',
    pattern: /^#[0-9a-fA-F]{6}$/,
  },
  {
    field: 'typography.fontSizeBase',
    required: true,
    type: 'string',
    pattern: /^\d+(\.\d+)?rem$/,
  },
  // ... more rules
];
```

### Accessibility Validation

```typescript
interface AccessibilityRule {
  wcagCriterion: string;
  description: string;
  level: 'A' | 'AA' | 'AAA';
  test: (element: HTMLElement) => boolean;
}

const accessibilityRules: AccessibilityRule[] = [
  {
    wcagCriterion: '1.4.3',
    description: 'Contrast (Minimum)',
    level: 'AA',
    test: (element) => {
      // Check contrast ratio >= 4.5:1 for normal text
    },
  },
  // ... more rules
];
```

## Migration Schema

### Version Compatibility

```typescript
interface MigrationSchema {
  from: string;
  to: string;
  transformations: {
    renameFields?: Record<string, string>;
    transformValues?: Record<string, (value: any) => any>;
    deprecatedFields?: string[];
    newFields?: Record<string, any>;
  };
}
```