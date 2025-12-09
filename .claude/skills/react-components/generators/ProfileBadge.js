const BaseGenerator = require('./BaseGenerator');

class ProfileBadgeGenerator extends BaseGenerator {
  async generate({ props_schema, styling, accessibility_level }) {
    const componentName = 'ProfileBadge';

    // Generate interfaces
    const interfaces = this.generateInterfaces(componentName, props_schema);

    // Generate component code
    const code = this.generateComponentCode(componentName, props_schema, styling, accessibility_level);

    // Generate styles
    const styles = this.generateProfileBadgeStyles(styling);

    // Generate tests
    const test = this.generateTest(componentName);

    // Generate Storybook story
    const story = this.generateStory(componentName);

    return {
      code: interfaces + '\n\n' + code,
      styles,
      test,
      story
    };
  }

  generateComponentCode(componentName, customProps, styling, accessibility) {
    const imports = this.generateImports({
      hasState: true,
      hasCallback: true,
      styling
    });

    const accessibilityAttrs = this.generateAccessibilityAttributes(accessibility);

    return `${imports}
const ${componentName}: React.FC<${componentName}Props> = ({
  user,
  showLevel = true,
  onClick,
  ...props
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  const handleToggle = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const handleMenuClick = useCallback(async (action: string) => {
    setIsLoading(true);
    setIsOpen(false);

    try {
      switch (action) {
        case 'profile':
          // Navigate to profile
          window.location.href = '/profile';
          break;
        case 'settings':
          // Navigate to settings
          window.location.href = '/settings';
          break;
        case 'logout':
          // Handle logout
          await fetch('/api/auth/logout', { method: 'POST' });
          window.location.href = '/login';
          break;
        case 'login':
          // Handle login
          if (onClick) {
            onClick();
          } else {
            window.location.href = '/login';
          }
          break;
      }
    } catch (err) {
      console.error('Error handling action:', err);
    } finally {
      setIsLoading(false);
    }
  }, [onClick]);

  const getInitials = (name: string) => {
    return name
      .split(' ')
      .map(part => part.charAt(0).toUpperCase())
      .join('')
      .substring(0, 2);
  };

  const getLevelColor = (level: string) => {
    const colors = {
      beginner: '#10b981',  // green
      intermediate: '#3b82f6',  // blue
      advanced: '#8b5cf6'  // purple
    };
    return colors[level as keyof typeof colors] || colors.intermediate;
  };

  const getLevelIcon = (level: string) => {
    const icons = {
      beginner: '🌱',
      intermediate: '🚀',
      advanced: '⭐'
    };
    return icons[level as keyof typeof icons] || '🚀';
  };

  // Close dropdown when clicking outside
  React.useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (isOpen && !e.target) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [isOpen]);

  // Guest user - show sign in button
  if (!user) {
    return (
      <button
        className={styles.signInButton}
        onClick={() => handleMenuClick('login')}
        disabled={isLoading}
        aria-label="Sign in to your account"
        aria-busy={isLoading}
        ${accessibilityAttrs.join(' ')}
      >
        {isLoading ? (
          <span className={styles.spinner} aria-hidden="true"></span>
        ) : null}
        Sign In
      </button>
    );
  }

  // Logged in user
  return (
    <div className={styles.container}>
      <button
        className={styles.badge}
        onClick={handleToggle}
        aria-expanded={isOpen}
        aria-haspopup="true"
        aria-label={\`Profile menu for \${user.name}\`}
        title={\`\${user.name} - \${user.experienceLevel} level\`}
        ${accessibilityAttrs.join(' ')}
      >
        {/* Avatar */}
        <div className={styles.avatar}>
          {user.avatar ? (
            <img
              src={user.avatar}
              alt={\`\${user.name}'s avatar\`}
              className={styles.avatarImage}
            />
          ) : (
            <div className={styles.avatarInitials}>
              {getInitials(user.name)}
            </div>
          )}
        </div>

        {/* User info */}
        <div className={styles.userInfo}>
          <span className={styles.userName}>{user.name}</span>
          {showLevel && (
            <div className={styles.level}>
              <span
                className={styles.levelBadge}
                style={{ backgroundColor: getLevelColor(user.experienceLevel) }}
              >
                {getLevelIcon(user.experienceLevel)}
                {user.experienceLevel}
              </span>
            </div>
          )}
        </div>

        {/* Dropdown arrow */}
        <svg
          className={\`\${styles.arrow} \${isOpen ? styles.arrowOpen : ''}\`}
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
          aria-hidden="true"
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            strokeWidth={2}
            d="M19 9l-7 7-7-7"
          />
        </svg>
      </button>

      {/* Dropdown menu */}
      {isOpen && (
        <div className={styles.dropdown} role="menu">
          <div className={styles.dropdownHeader}>
            <div className={styles.headerAvatar}>
              {user.avatar ? (
                <img
                  src={user.avatar}
                  alt={\`\${user.name}'s avatar\`}
                  className={styles.headerAvatarImage}
                />
              ) : (
                <div className={styles.headerAvatarInitials}>
                  {getInitials(user.name)}
                </div>
              )}
            </div>
            <div className={styles.headerInfo}>
              <div className={styles.headerName}>{user.name}</div>
              <div className={styles.headerEmail}>{user.email}</div>
              {showLevel && (
                <div className={styles.headerLevel}>
                  Level: <span style={{ color: getLevelColor(user.experienceLevel) }}>
                    {user.experienceLevel}
                  </span>
                </div>
              )}
            </div>
          </div>

          <div className={styles.divider} />

          <div className={styles.menuItems}>
            <button
              className={styles.menuItem}
              onClick={() => handleMenuClick('profile')}
              role="menuitem"
            >
              <svg className={styles.menuIcon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M16 7a4 4 0 11-8 0 4 4 0 018 0zM12 14a7 7 0 00-7 7h14a7 7 0 00-7-7z" />
              </svg>
              View Profile
            </button>

            <button
              className={styles.menuItem}
              onClick={() => handleMenuClick('settings')}
              role="menuitem"
            >
              <svg className={styles.menuIcon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
              </svg>
              Settings
            </button>

            <div className={styles.divider} />

            <button
              className={\`\${styles.menuItem} \${styles.logout}\`}
              onClick={() => handleMenuClick('logout')}
              role="menuitem"
            >
              <svg className={styles.menuIcon} fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M17 16l4-4m0 0l-4-4m4 4H7m6 4v1a3 3 0 01-3 3H6a3 3 0 01-3-3V7a3 3 0 013-3h4a3 3 0 013 3v1" />
              </svg>
              Sign Out
            </button>
          </div>
        </div>
      )}

      {/* Overlay for mobile */}
      {isOpen && (
        <div className={styles.overlay} onClick={handleToggle} />
      )}
    </div>
  );
};

export default ${componentName};`;
  }

  generateProfileBadgeStyles(styling) {
    if (styling !== 'css_modules') return '';

    return `
.container {
  position: relative;
  display: inline-block;
}

/* Sign in button */
.signInButton {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  background: #3b82f6;
  color: white;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease;
  min-height: 44px;
  min-width: 44px;
}

.signInButton:hover {
  background: #2563eb;
  transform: translateY(-1px);
  box-shadow: 0 4px 12px rgba(59, 130, 246, 0.3);
}

.signInButton:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

.signInButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.spinner {
  width: 16px;
  height: 16px;
  border: 2px solid rgba(255, 255, 255, 0.3);
  border-top-color: white;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

/* Profile badge */
.badge {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 6px;
  background: transparent;
  border: 2px solid transparent;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s ease;
  min-height: 44px;
}

.badge:hover {
  background: rgba(59, 130, 246, 0.05);
  border-color: rgba(59, 130, 246, 0.2);
}

.badge:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

.avatar {
  position: relative;
  width: 40px;
  height: 40px;
  flex-shrink: 0;
}

.avatarImage,
.headerAvatarImage {
  width: 100%;
  height: 100%;
  border-radius: 50%;
  object-fit: cover;
}

.avatarInitials,
.headerAvatarInitials {
  width: 100%;
  height: 100%;
  border-radius: 50%;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  display: flex;
  align-items: center;
  justify-content: center;
  color: white;
  font-weight: 600;
  font-size: 14px;
}

.userInfo {
  display: flex;
  flex-direction: column;
  gap: 2px;
  min-width: 0;
}

.userName {
  font-size: 14px;
  font-weight: 600;
  color: #111827;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.level {
  display: flex;
  align-items: center;
}

.levelBadge {
  display: inline-flex;
  align-items: center;
  gap: 4px;
  padding: 2px 8px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: 600;
  color: white;
  white-space: nowrap;
}

.arrow {
  width: 16px;
  height: 16px;
  color: #6b7280;
  transition: transform 0.2s ease;
  margin-left: 4px;
}

.arrowOpen {
  transform: rotate(180deg);
}

/* Dropdown */
.dropdown {
  position: absolute;
  top: calc(100% + 8px);
  right: 0;
  min-width: 280px;
  background: white;
  border: 1px solid #e5e7eb;
  border-radius: 12px;
  box-shadow: 0 10px 40px rgba(0, 0, 0, 0.1);
  z-index: 1000;
  overflow: hidden;
  animation: dropdownAppear 0.2s ease;
}

@keyframes dropdownAppear {
  from {
    opacity: 0;
    transform: translateY(-4px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.dropdownHeader {
  padding: 16px;
  display: flex;
  align-items: center;
  gap: 12px;
}

.headerAvatar {
  width: 48px;
  height: 48px;
  flex-shrink: 0;
}

.headerInfo {
  flex: 1;
  min-width: 0;
}

.headerName {
  font-size: 16px;
  font-weight: 600;
  color: #111827;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.headerEmail {
  font-size: 13px;
  color: #6b7280;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.headerLevel {
  font-size: 12px;
  color: #6b7280;
  margin-top: 4px;
}

.divider {
  height: 1px;
  background: #e5e7eb;
  margin: 0 8px;
}

.menuItems {
  padding: 4px;
}

.menuItem {
  display: flex;
  align-items: center;
  gap: 12px;
  width: 100%;
  padding: 10px 12px;
  background: transparent;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  color: #374151;
  cursor: pointer;
  transition: all 0.2s ease;
  text-align: left;
  min-height: 44px;
}

.menuItem:hover {
  background: #f3f4f6;
  color: #111827;
}

.menuItem:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

.menuItem.logout {
  color: #dc2626;
}

.menuItem.logout:hover {
  background: #fef2f2;
  color: #b91c1c;
}

.menuIcon {
  width: 18px;
  height: 18px;
  color: #6b7280;
  flex-shrink: 0;
}

.menuItem.logout .menuIcon {
  color: #dc2626;
}

.overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  z-index: 999;
}

/* Dark theme */
[data-theme='dark'] .badge:hover {
  background: rgba(59, 130, 246, 0.1);
  border-color: rgba(59, 130, 246, 0.3);
}

[data-theme='dark'] .userName {
  color: #f9fafb;
}

[data-theme='dark'] .dropdown {
  background: #1f2937;
  border-color: #374151;
}

[data-theme='dark'] .dropdownHeader {
  border-bottom-color: #374151;
}

[data-theme='dark'] .headerName {
  color: #f9fafb;
}

[data-theme='dark'] .headerEmail {
  color: #9ca3af;
}

[data-theme='dark'] .headerLevel {
  color: #9ca3af;
}

[data-theme='dark'] .divider {
  background: #374151;
}

[data-theme='dark'] .menuItem {
  color: #d1d5db;
}

[data-theme='dark'] .menuItem:hover {
  background: #374151;
  color: #f9fafb;
}

[data-theme='dark'] .menuItem.logout {
  color: #f87171;
}

[data-theme='dark'] .menuItem.logout:hover {
  background: #7f1d1d;
  color: #fca5a5;
}

[data-theme='dark'] .menuIcon {
  color: #9ca3af;
}

[data-theme='dark'] .menuItem.logout .menuIcon {
  color: #f87171;
}

/* Mobile responsive */
@media (max-width: 640px) {
  .badge {
    padding: 4px;
  }

  .userInfo {
    display: none;
  }

  .arrow {
    margin-left: 0;
  }

  .dropdown {
    position: fixed;
    top: auto;
    bottom: 0;
    left: 0;
    right: 0;
    width: 100%;
    min-width: auto;
    border-radius: 12px 12px 0 0;
    animation: slideUp 0.3s ease;
  }

  @keyframes slideUp {
    from {
      transform: translateY(100%);
    }
    to {
      transform: translateY(0);
    }
  }

  .dropdownHeader {
    padding: 20px 16px;
  }

  .menuItems {
    padding: 8px;
  }

  .menuItem {
    padding: 14px 12px;
    font-size: 16px;
  }
}

/* High contrast mode */
@media (prefers-contrast: high) {
  .badge {
    border-color: currentColor;
  }

  .dropdown {
    border-width: 2px;
    border-color: currentColor;
  }

  .menuItem {
    border: 1px solid transparent;
  }

  .menuItem:hover {
    border-color: currentColor;
  }
}

/* Reduced motion */
@media (prefers-reduced-motion: reduce) {
  .dropdown,
  .arrow {
    animation: none;
    transition: none;
  }
}`;
  }

  generateTest(componentName) {
    return `import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import '@testing-library/jest-dom';
import ${componentName} from './index';

// Mock fetch
global.fetch = jest.fn();

const mockFetch = fetch as jest.MockedFunction<typeof fetch>;

describe('${componentName}', () => {
  const mockUser = {
    id: '1',
    name: 'John Doe',
    email: 'john@example.com',
    experienceLevel: 'intermediate' as const
  };

  beforeEach(() => {
    mockFetch.mockClear();
  });

  it('renders sign in button when no user is provided', () => {
    render(
      <${componentName}
        user={null}
        onClick={() => {}}
      />
    );

    expect(screen.getByRole('button', { name: /sign in to your account/i })).toBeInTheDocument();
    expect(screen.getByText('Sign In')).toBeInTheDocument();
  });

  it('calls onClick when sign in button is clicked', async () => {
    const mockOnClick = jest.fn();
    const user = userEvent.setup();

    render(
      <${componentName}
        user={null}
        onClick={mockOnClick}
      />
    );

    const signInButton = screen.getByRole('button', { name: /sign in to your account/i });
    await user.click(signInButton);

    expect(mockOnClick).toHaveBeenCalled();
  });

  it('renders user badge when user is provided', () => {
    render(
      <${componentName}
        user={mockUser}
      />
    );

    expect(screen.getByText('John Doe')).toBeInTheDocument();
    expect(screen.getByText('intermediate')).toBeInTheDocument();
    expect(screen.getByLabelText(/profile menu for john doe/i)).toBeInTheDocument();
  });

  it('shows user initials when no avatar is provided', () => {
    render(
      <${componentName}
        user={mockUser}
      />
    );

    expect(screen.getByText('JD')).toBeInTheDocument();
  });

  it('shows user avatar when provided', () => {
    const userWithAvatar = {
      ...mockUser,
      avatar: 'https://example.com/avatar.jpg'
    };

    render(
      <${componentName}
        user={userWithAvatar}
      />
    );

    const avatar = screen.getByAltText("John Doe's avatar");
    expect(avatar).toBeInTheDocument();
    expect(avatar).toHaveAttribute('src', 'https://example.com/avatar.jpg');
  });

  it('hides level when showLevel is false', () => {
    render(
      <${componentName}
        user={mockUser}
        showLevel={false}
      />
    );

    expect(screen.queryByText('intermediate')).not.toBeInTheDocument();
  });

  it('opens dropdown menu when badge is clicked', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        user={mockUser}
      />
    );

    const badge = screen.getByRole('button', { name: /profile menu for john doe/i });
    await user.click(badge);

    expect(screen.getByRole('menu')).toBeInTheDocument();
    expect(screen.getByRole('menuitem', { name: /view profile/i })).toBeInTheDocument();
    expect(screen.getByRole('menuitem', { name: /settings/i })).toBeInTheDocument();
    expect(screen.getByRole('menuitem', { name: /sign out/i })).toBeInTheDocument();
  });

  it('shows user details in dropdown header', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        user={mockUser}
      />
    );

    const badge = screen.getByRole('button', { name: /profile menu for john doe/i });
    await user.click(badge);

    expect(screen.getByText('John Doe')).toBeInTheDocument();
    expect(screen.getByText('john@example.com')).toBeInTheDocument();
    expect(screen.getByText(/level:/i)).toBeInTheDocument();
    expect(screen.getByText('intermediate')).toBeInTheDocument();
  });

  it('closes dropdown when clicking outside', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        user={mockUser}
      />
    );

    const badge = screen.getByRole('button', { name: /profile menu for john doe/i });
    await user.click(badge);

    expect(screen.getByRole('menu')).toBeInTheDocument();

    // Click outside
    fireEvent.mouseDown(document.body);

    await waitFor(() => {
      expect(screen.queryByRole('menu')).not.toBeInTheDocument();
    });
  });

  it('handles logout action', async () => {
    const user = userEvent.setup();
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({})
    });

    // Mock window.location
    delete (window as any).location;
    window.location = { href: '' } as any;

    render(
      <${componentName}
        user={mockUser}
      />
    );

    const badge = screen.getByRole('button', { name: /profile menu for john doe/i });
    await user.click(badge);

    const logoutButton = screen.getByRole('menuitem', { name: /sign out/i });
    await user.click(logoutButton);

    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/auth/logout', { method: 'POST' });
    });

    await waitFor(() => {
      expect(window.location.href).toBe('/login');
    });
  });

  it('displays correct level colors', async () => {
    const user = userEvent.setup();
    const advancedUser = {
      ...mockUser,
      experienceLevel: 'advanced' as const
    };

    render(
      <${componentName}
        user={advancedUser}
      />
    );

    const badge = screen.getByRole('button', { name: /profile menu for john doe/i });
    await user.click(badge);

    const levelBadge = screen.getByText('advanced').parentElement;
    expect(levelBadge).toHaveStyle('background-color: rgb(139, 92, 246)'); // purple
  });

  it('shows loading state during actions', async () => {
    const user = userEvent.setup();
    mockFetch.mockImplementationOnce(() => new Promise(resolve => setTimeout(resolve, 100)));

    render(
      <${componentName}
        user={null}
        onClick={() => {}}
      />
    );

    const signInButton = screen.getByRole('button', { name: /sign in to your account/i });
    await user.click(signInButton);

    expect(signInButton).toBeDisabled();
    expect(signInButton).toHaveAttribute('aria-busy', 'true');
  });

  it('is keyboard accessible', async () => {
    render(
      <${componentName}
        user={mockUser}
      />
    );

    const badge = screen.getByRole('button', { name: /profile menu for john doe/i });

    // Focus with Tab
    badge.focus();
    expect(badge).toHaveFocus();

    // Open with Enter
    fireEvent.keyDown(badge, { key: 'Enter' });

    await waitFor(() => {
      expect(screen.getByRole('menu')).toBeInTheDocument();
    });
  });
});`;
  }

  generateStory(componentName) {
    return `import type { Meta, StoryObj } from '@storybook/react';
import { ${componentName} } from './index';

const meta: Meta<typeof ${componentName}> = {
  title: 'Components/${componentName}',
  component: ${componentName},
  parameters: {
    layout: 'centered',
  },
  tags: ['autodocs'],
  argTypes: {
    showLevel: {
      control: 'boolean',
      description: 'Whether to show the experience level badge',
    },
  },
  decorators: [
    (Story) => (
      <div style={{ padding: '20px', background: '#f9fafb', borderRadius: '8px' }}>
        <Story />
      </div>
    ),
  ],
};

export default meta;
type Story = StoryObj<typeof meta>;

export const SignedOut: Story = {
  args: {
    user: null,
    onClick: () => console.log('Sign in clicked'),
  },
};

export const SignedIn: Story = {
  args: {
    user: {
      id: '1',
      name: 'Sarah Johnson',
      email: 'sarah@example.com',
      experienceLevel: 'intermediate',
    },
  },
};

export const Beginner: Story = {
  args: {
    user: {
      id: '2',
      name: 'Alex Chen',
      email: 'alex@example.com',
      experienceLevel: 'beginner',
    },
  },
};

export const Advanced: Story = {
  args: {
    user: {
      id: '3',
      name: 'Dr. Michael Roberts',
      email: 'michael@techcorp.com',
      experienceLevel: 'advanced',
    },
  },
};

export const WithAvatar: Story = {
  args: {
    user: {
      id: '4',
      name: 'Emma Wilson',
      email: 'emma@example.com',
      experienceLevel: 'intermediate',
      avatar: 'https://api.dicebear.com/7.x/avataaars/svg?seed=Emma',
    },
  },
};

export const WithoutLevel: Story = {
  args: {
    user: {
      id: '5',
      name: 'James Taylor',
      email: 'james@example.com',
      experienceLevel: 'intermediate',
    },
    showLevel: false,
  },
};

export const WithCallback: Story = {
  args: {
    user: null,
    onClick: () => {
      alert('Opening sign in modal...');
    },
  },
};`;
  }
}

module.exports = ProfileBadgeGenerator;