# UI/UX Polish Recipes

Copy-paste friendly patterns for common UI enhancements. Adapt to your tech stack.

---

## 📱 Responsive Navbar

**When to use:** Desktop navbar needs to collapse to hamburger menu on mobile.

### React + Tailwind + Framer Motion

```tsx
import { useState } from 'react';
import { motion, AnimatePresence } from 'framer-motion';

export function Navbar() {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <nav className="bg-white shadow-sm">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between items-center h-16">
          {/* Logo */}
          <div className="flex-shrink-0">
            <a href="/" className="text-xl font-bold text-gray-900">
              Logo
            </a>
          </div>

          {/* Desktop Menu */}
          <div className="hidden md:flex space-x-8">
            <a href="/features" className="text-gray-700 hover:text-gray-900">
              Features
            </a>
            <a href="/pricing" className="text-gray-700 hover:text-gray-900">
              Pricing
            </a>
            <a href="/about" className="text-gray-700 hover:text-gray-900">
              About
            </a>
          </div>

          {/* Mobile Hamburger */}
          <button
            className="md:hidden p-2 rounded-md text-gray-700 hover:bg-gray-100"
            onClick={() => setIsOpen(!isOpen)}
            aria-expanded={isOpen}
            aria-controls="mobile-menu"
            aria-label="Toggle menu"
          >
            <svg
              className="h-6 w-6"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              {isOpen ? (
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M6 18L18 6M6 6l12 12"
                />
              ) : (
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M4 6h16M4 12h16M4 18h16"
                />
              )}
            </svg>
          </button>
        </div>
      </div>

      {/* Mobile Menu */}
      <AnimatePresence>
        {isOpen && (
          <motion.div
            id="mobile-menu"
            initial={{ height: 0, opacity: 0 }}
            animate={{ height: 'auto', opacity: 1 }}
            exit={{ height: 0, opacity: 0 }}
            transition={{ duration: 0.2, ease: 'easeOut' }}
            className="md:hidden overflow-hidden"
          >
            <div className="px-2 pt-2 pb-3 space-y-1">
              <a
                href="/features"
                className="block px-3 py-2 rounded-md text-gray-700 hover:bg-gray-100"
              >
                Features
              </a>
              <a
                href="/pricing"
                className="block px-3 py-2 rounded-md text-gray-700 hover:bg-gray-100"
              >
                Pricing
              </a>
              <a
                href="/about"
                className="block px-3 py-2 rounded-md text-gray-700 hover:bg-gray-100"
              >
                About
              </a>
            </div>
          </motion.div>
        )}
      </AnimatePresence>
    </nav>
  );
}
```

### CSS-Only Fallback (No Framer Motion)

```tsx
// Add this CSS
.mobile-menu {
  max-height: 0;
  overflow: hidden;
  transition: max-height 0.2s ease-out, opacity 0.2s ease-out;
  opacity: 0;
}

.mobile-menu.open {
  max-height: 500px; /* Large enough to fit content */
  opacity: 1;
}

@media (prefers-reduced-motion: reduce) {
  .mobile-menu {
    transition: none;
  }
}

// Component
<div className={`mobile-menu ${isOpen ? 'open' : ''}`}>
  {/* Menu content */}
</div>
```

---

## 🎯 Side Drawer with Overlay

**When to use:** Side navigation or settings panel that slides in from the side.

### React + Tailwind + Framer Motion

```tsx
import { motion, AnimatePresence } from 'framer-motion';
import { useEffect } from 'react';

interface SideDrawerProps {
  isOpen: boolean;
  onClose: () => void;
  children: React.ReactNode;
}

export function SideDrawer({ isOpen, onClose, children }: SideDrawerProps) {
  // Close on Escape key
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    if (isOpen) {
      document.addEventListener('keydown', handleEscape);
      // Prevent body scroll when drawer open
      document.body.style.overflow = 'hidden';
    }
    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.body.style.overflow = '';
    };
  }, [isOpen, onClose]);

  return (
    <AnimatePresence>
      {isOpen && (
        <>
          {/* Overlay */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
            className="fixed inset-0 bg-black/50 z-40"
            onClick={onClose}
            aria-hidden="true"
          />

          {/* Drawer */}
          <motion.div
            initial={{ x: '-100%' }}
            animate={{ x: 0 }}
            exit={{ x: '-100%' }}
            transition={{ duration: 0.25, ease: 'easeOut' }}
            className="fixed inset-y-0 left-0 w-64 bg-white shadow-xl z-50 overflow-y-auto"
            role="dialog"
            aria-modal="true"
          >
            <div className="p-4">
              <button
                onClick={onClose}
                className="mb-4 p-2 rounded-md hover:bg-gray-100"
                aria-label="Close drawer"
              >
                <svg
                  className="h-6 w-6"
                  fill="none"
                  viewBox="0 0 24 24"
                  stroke="currentColor"
                >
                  <path
                    strokeLinecap="round"
                    strokeLinejoin="round"
                    strokeWidth={2}
                    d="M6 18L18 6M6 6l12 12"
                  />
                </svg>
              </button>
              {children}
            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

// Usage
<SideDrawer isOpen={isOpen} onClose={() => setIsOpen(false)}>
  <nav className="space-y-2">
    <a href="/dashboard" className="block px-4 py-2 rounded hover:bg-gray-100">
      Dashboard
    </a>
    <a href="/settings" className="block px-4 py-2 rounded hover:bg-gray-100">
      Settings
    </a>
  </nav>
</SideDrawer>
```

---

## 📂 Submenu Expand/Collapse Animation

**When to use:** Nested navigation menu with expandable sections.

### React + Framer Motion

```tsx
import { motion, AnimatePresence } from 'framer-motion';
import { useState } from 'react';

interface MenuItemProps {
  label: string;
  icon?: React.ReactNode;
  submenu?: { label: string; href: string }[];
}

export function MenuItem({ label, icon, submenu }: MenuItemProps) {
  const [isExpanded, setIsExpanded] = useState(false);

  return (
    <div>
      <button
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
        aria-controls={submenu ? `submenu-${label}` : undefined}
        className="w-full flex items-center justify-between px-4 py-2 text-gray-700 hover:bg-gray-100 rounded-md transition-colors"
      >
        <div className="flex items-center gap-3">
          {icon}
          <span>{label}</span>
        </div>
        {submenu && (
          <motion.svg
            animate={{ rotate: isExpanded ? 180 : 0 }}
            transition={{ duration: 0.2 }}
            className="h-5 w-5 text-gray-500"
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M19 9l-7 7-7-7"
            />
          </motion.svg>
        )}
      </button>

      {submenu && (
        <AnimatePresence initial={false}>
          {isExpanded && (
            <motion.div
              id={`submenu-${label}`}
              initial={{ height: 0, opacity: 0 }}
              animate={{ height: 'auto', opacity: 1 }}
              exit={{ height: 0, opacity: 0 }}
              transition={{ duration: 0.2, ease: 'easeOut' }}
              className="overflow-hidden"
            >
              <div className="pl-12 pr-4 py-2 space-y-1">
                {submenu.map((item) => (
                  <a
                    key={item.href}
                    href={item.href}
                    className="block px-3 py-2 text-sm text-gray-600 hover:text-gray-900 hover:bg-gray-50 rounded-md"
                  >
                    {item.label}
                  </a>
                ))}
              </div>
            </motion.div>
          )}
        </AnimatePresence>
      )}
    </div>
  );
}

// Usage
<MenuItem
  label="Products"
  submenu={[
    { label: 'All Products', href: '/products' },
    { label: 'Categories', href: '/products/categories' },
    { label: 'New Arrivals', href: '/products/new' },
  ]}
/>
```

### CSS-Only Version

```tsx
// CSS
.submenu {
  display: grid;
  grid-template-rows: 0fr;
  transition: grid-template-rows 0.2s ease-out, opacity 0.2s ease-out;
  opacity: 0;
}

.submenu.open {
  grid-template-rows: 1fr;
  opacity: 1;
}

.submenu > div {
  overflow: hidden;
}

.chevron {
  transition: transform 0.2s ease-out;
}

.chevron.rotated {
  transform: rotate(180deg);
}

@media (prefers-reduced-motion: reduce) {
  .submenu,
  .chevron {
    transition: none;
  }
}

// Component
<button
  onClick={() => setIsExpanded(!isExpanded)}
  aria-expanded={isExpanded}
>
  {label}
  <span className={`chevron ${isExpanded ? 'rotated' : ''}`}>▼</span>
</button>

<div className={`submenu ${isExpanded ? 'open' : ''}`}>
  <div>
    {/* Submenu items */}
  </div>
</div>
```

---

## ♿ Accessible Disclosure Menu Pattern

**When to use:** Menu that requires WCAG-compliant keyboard navigation and ARIA.

```tsx
import { useState, useRef, useEffect } from 'react';

export function AccessibleMenu() {
  const [isOpen, setIsOpen] = useState(false);
  const buttonRef = useRef<HTMLButtonElement>(null);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close on Escape, return focus
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        setIsOpen(false);
        buttonRef.current?.focus();
      }
    };

    // Close on outside click
    const handleClickOutside = (e: MouseEvent) => {
      if (
        menuRef.current &&
        !menuRef.current.contains(e.target as Node) &&
        !buttonRef.current?.contains(e.target as Node)
      ) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleEscape);
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('keydown', handleEscape);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  return (
    <div className="relative">
      <button
        ref={buttonRef}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
        aria-controls="menu-dropdown"
        className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2"
      >
        Menu
      </button>

      {isOpen && (
        <div
          ref={menuRef}
          id="menu-dropdown"
          role="menu"
          aria-labelledby="menu-button"
          className="absolute right-0 mt-2 w-56 rounded-md shadow-lg bg-white ring-1 ring-black ring-opacity-5"
        >
          <div className="py-1">
            <a
              href="/account"
              role="menuitem"
              className="block px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 focus:outline-none focus:bg-gray-100"
            >
              Account Settings
            </a>
            <a
              href="/support"
              role="menuitem"
              className="block px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 focus:outline-none focus:bg-gray-100"
            >
              Support
            </a>
            <a
              href="/license"
              role="menuitem"
              className="block px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 focus:outline-none focus:bg-gray-100"
            >
              License
            </a>
            <button
              role="menuitem"
              className="w-full text-left block px-4 py-2 text-sm text-gray-700 hover:bg-gray-100 focus:outline-none focus:bg-gray-100"
              onClick={() => {
                console.log('Sign out');
                setIsOpen(false);
              }}
            >
              Sign Out
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
```

---

## 🎨 Micro-Interactions: Button Hover

**When to use:** Add subtle premium feel to buttons.

### Tailwind CSS

```tsx
// Lift effect (shadow + translate)
<button className="px-4 py-2 bg-blue-600 text-white rounded-md transition-all duration-200 hover:bg-blue-700 hover:shadow-lg hover:-translate-y-0.5 active:translate-y-0">
  Click me
</button>

// Scale effect
<button className="px-4 py-2 bg-blue-600 text-white rounded-md transition-transform duration-200 hover:scale-105 active:scale-100">
  Click me
</button>

// Glow effect
<button className="px-4 py-2 bg-blue-600 text-white rounded-md transition-shadow duration-200 hover:shadow-[0_0_20px_rgba(59,130,246,0.5)]">
  Click me
</button>
```

### Reduced Motion Support

```tsx
<button className="px-4 py-2 bg-blue-600 text-white rounded-md transition-all duration-200 hover:bg-blue-700 hover:shadow-lg hover:-translate-y-0.5 motion-reduce:transition-none motion-reduce:hover:translate-y-0">
  Accessible Button
</button>
```

---

## 🔗 Link Underline Animation

**When to use:** Premium hover effect for text links.

### CSS

```css
.animated-link {
  position: relative;
  text-decoration: none;
  color: #2563eb; /* blue-600 */
}

.animated-link::after {
  content: '';
  position: absolute;
  bottom: 0;
  left: 0;
  width: 0;
  height: 2px;
  background-color: #2563eb;
  transition: width 0.2s ease-out;
}

.animated-link:hover::after {
  width: 100%;
}

@media (prefers-reduced-motion: reduce) {
  .animated-link::after {
    transition: none;
  }
}
```

### Tailwind CSS (Using @apply)

```css
@layer components {
  .link-animated {
    @apply relative text-blue-600 no-underline;
  }

  .link-animated::after {
    @apply content-[''] absolute bottom-0 left-0 w-0 h-0.5 bg-blue-600 transition-all duration-200;
  }

  .link-animated:hover::after {
    @apply w-full;
  }

  @media (prefers-reduced-motion: reduce) {
    .link-animated::after {
      @apply transition-none;
    }
  }
}
```

---

## 🃏 Card Hover Effect

**When to use:** Interactive cards in grid layouts.

```tsx
<div className="group relative overflow-hidden rounded-lg border border-gray-200 bg-white transition-all duration-200 hover:shadow-xl hover:-translate-y-1 motion-reduce:transition-none motion-reduce:hover:translate-y-0">
  <img
    src="/image.jpg"
    alt="Card image"
    className="w-full h-48 object-cover transition-transform duration-300 group-hover:scale-105 motion-reduce:transition-none motion-reduce:group-hover:scale-100"
  />
  <div className="p-6">
    <h3 className="text-xl font-semibold text-gray-900">Card Title</h3>
    <p className="mt-2 text-gray-600">Card description goes here.</p>
  </div>
</div>
```

---

## 🎭 Loading Skeleton

**When to use:** Content is loading, show placeholder.

```tsx
export function SkeletonCard() {
  return (
    <div className="animate-pulse">
      <div className="h-48 bg-gray-200 rounded-t-lg"></div>
      <div className="p-6 space-y-3">
        <div className="h-6 bg-gray-200 rounded w-3/4"></div>
        <div className="h-4 bg-gray-200 rounded w-full"></div>
        <div className="h-4 bg-gray-200 rounded w-5/6"></div>
      </div>
    </div>
  );
}

// CSS for custom skeleton shimmer
@keyframes shimmer {
  0% {
    background-position: -1000px 0;
  }
  100% {
    background-position: 1000px 0;
  }
}

.skeleton-shimmer {
  background: linear-gradient(
    90deg,
    #f0f0f0 0%,
    #e0e0e0 50%,
    #f0f0f0 100%
  );
  background-size: 1000px 100%;
  animation: shimmer 2s infinite;
}

@media (prefers-reduced-motion: reduce) {
  .skeleton-shimmer {
    animation: none;
  }
}
```

---

## 📏 Consistent Spacing System

**When to use:** Enforce spacing consistency across the app.

### Tailwind Config

```js
// tailwind.config.js
module.exports = {
  theme: {
    spacing: {
      0: '0',
      1: '4px',
      2: '8px',
      3: '12px',
      4: '16px',
      5: '20px',
      6: '24px',
      8: '32px',
      10: '40px',
      12: '48px',
      16: '64px',
      20: '80px',
      24: '96px',
    },
  },
};
```

### CSS Custom Properties

```css
:root {
  --space-1: 4px;
  --space-2: 8px;
  --space-3: 12px;
  --space-4: 16px;
  --space-5: 20px;
  --space-6: 24px;
  --space-8: 32px;
  --space-10: 40px;
  --space-12: 48px;
  --space-16: 64px;
}

/* Usage */
.container {
  padding: var(--space-6);
  gap: var(--space-4);
}
```

---

## 🎯 Focus Ring Consistency

**When to use:** Ensure all focusable elements have visible focus indicators.

### Tailwind CSS

```tsx
// Global focus ring style
<button className="px-4 py-2 bg-blue-600 text-white rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 focus:ring-offset-2">
  Button
</button>

// Use :focus-visible (only show on keyboard focus, not mouse click)
<button className="px-4 py-2 bg-blue-600 text-white rounded-md focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-blue-500 focus-visible:ring-offset-2">
  Better Button
</button>
```

### Global CSS

```css
/* Remove default outline, add custom ring */
*:focus {
  outline: none;
}

*:focus-visible {
  outline: 2px solid #3b82f6; /* blue-500 */
  outline-offset: 2px;
}

/* Or use box-shadow for more control */
*:focus-visible {
  outline: none;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.5);
}
```

---

**End of Recipes**

Adapt these patterns to your project's tech stack and design system. Always test for accessibility and reduced motion support.
