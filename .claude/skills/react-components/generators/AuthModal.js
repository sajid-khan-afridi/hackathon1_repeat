const BaseGenerator = require('./BaseGenerator');

class AuthModalGenerator extends BaseGenerator {
  async generate({ props_schema, styling, accessibility_level }) {
    const componentName = 'AuthModal';

    // Generate interfaces
    const interfaces = this.generateInterfaces(componentName, props_schema);

    // Generate component code
    const code = this.generateComponentCode(componentName, props_schema, styling, accessibility_level);

    // Generate styles
    const styles = this.generateAuthModalStyles(styling);

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
      hasEffects: true,
      hasRef: true,
      styling
    });

    const accessibilityAttrs = this.generateAccessibilityAttributes(accessibility);

    return `${imports}
interface FormData {
  email: string;
  password: string;
  confirmPassword?: string;
  firstName?: string;
  lastName?: string;
  experienceLevel?: 'beginner' | 'intermediate' | 'advanced';
}

interface ProfileQuestions {
  experienceLevel: string;
  learningGoal: string;
  programmingBackground: string;
  timeCommitment: string;
  preferredTopics: string[];
}

const ${componentName}: React.FC<${componentName}Props> = ({
  mode = 'signin',
  onSuccess,
  onClose,
  providers = ['google', 'github', 'microsoft'],
  ...props
}) => {
  const [currentMode, setCurrentMode] = useState<'signin' | 'signup'>(mode);
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
    experienceLevel: 'intermediate'
  });
  const [profileQuestions, setProfileQuestions] = useState<ProfileQuestions>({
    experienceLevel: '',
    learningGoal: '',
    programmingBackground: '',
    timeCommitment: '',
    preferredTopics: []
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const modalRef = useRef<HTMLDivElement>(null);

  // Focus management
  useEffect(() => {
    if (modalRef.current) {
      modalRef.current.focus();
    }
  }, []);

  // Trap focus within modal
  useEffect(() => {
    const handleTabKey = (e: KeyboardEvent) => {
      if (e.key !== 'Tab') return;

      const focusableElements = modalRef.current?.querySelectorAll(
        'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
      ) as NodeListOf<HTMLElement>;

      if (!focusableElements.length) return;

      const firstElement = focusableElements[0];
      const lastElement = focusableElements[focusableElements.length - 1];

      if (e.shiftKey) {
        if (document.activeElement === firstElement) {
          lastElement.focus();
          e.preventDefault();
        }
      } else {
        if (document.activeElement === lastElement) {
          firstElement.focus();
          e.preventDefault();
        }
      }
    };

    const handleEscapeKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
      }
    };

    document.addEventListener('keydown', handleTabKey);
    document.addEventListener('keydown', handleEscapeKey);

    return () => {
      document.removeEventListener('keydown', handleTabKey);
      document.removeEventListener('keydown', handleEscapeKey);
    };
  }, [onClose]);

  const validateEmail = (email: string) => {
    const emailRegex = /^[^\\s@]+@[^\\s@]+\\.[^\\s@]+$/;
    return emailRegex.test(email);
  };

  const validatePassword = (password: string) => {
    return password.length >= 8;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (currentMode === 'signup' && step === 1) {
      // Validate form data
      if (!validateEmail(formData.email)) {
        setError('Please enter a valid email address');
        return;
      }

      if (!validatePassword(formData.password)) {
        setError('Password must be at least 8 characters long');
        return;
      }

      if (formData.password !== formData.confirmPassword) {
        setError('Passwords do not match');
        return;
      }

      setStep(2);
      return;
    }

    setIsLoading(true);

    try {
      const endpoint = currentMode === 'signin' ? '/api/auth/signin' : '/api/auth/signup';
      const payload = currentMode === 'signin'
        ? { email: formData.email, password: formData.password }
        : {
            ...formData,
            profile: profileQuestions
          };

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Authentication failed');
      }

      const { user } = await response.json();
      onSuccess(user);
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  };

  const handleOAuth = async (provider: string) => {
    setIsLoading(true);
    setError(null);

    try {
      // In a real app, this would redirect to OAuth provider
      window.location.href = \`/api/auth/\${provider}\`;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'OAuth authentication failed');
      setIsLoading(false);
    }
  };

  const handleInputChange = (field: keyof FormData) => (
    e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>
  ) => {
    setFormData(prev => ({ ...prev, [field]: e.target.value }));
  };

  const handleProfileChange = (field: keyof ProfileQuestions) => (
    e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>
  ) => {
    if (field === 'preferredTopics') {
      const target = e.target as HTMLInputElement;
      if (target.checked) {
        setProfileQuestions(prev => ({
          ...prev,
          [field]: [...prev[field], target.value]
        }));
      } else {
        setProfileQuestions(prev => ({
          ...prev,
          [field]: prev[field].filter(topic => topic !== target.value)
        }));
      }
    } else {
      setProfileQuestions(prev => ({ ...prev, [field]: e.target.value }));
    }
  };

  const switchMode = (newMode: 'signin' | 'signup') => {
    setCurrentMode(newMode);
    setStep(1);
    setError(null);
    setFormData({
      email: '',
      password: '',
      confirmPassword: '',
      firstName: '',
      lastName: '',
      experienceLevel: 'intermediate'
    });
  };

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div
        className={styles.modal}
        onClick={(e) => e.stopPropagation()}
        ref={modalRef}
        tabIndex={-1}
        role="dialog"
        aria-modal="true"
        aria-labelledby="modal-title"
        ${accessibilityAttrs.join(' ')}
      >
        {/* Close button */}
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close authentication modal"
        >
          ×
        </button>

        {/* Modal content */}
        <div className={styles.content}>
          {/* Header */}
          <div className={styles.header}>
            <h2 id="modal-title" className={styles.title}>
              {currentMode === 'signin' ? 'Sign In' : 'Create Account'}
            </h2>
            <p className={styles.subtitle}>
              {currentMode === 'signin'
                ? 'Welcome back! Please sign in to continue.'
                : 'Join our learning community today.'
              }
            </p>
          </div>

          {/* Error message */}
          {error && (
            <div className={styles.errorMessage} role="alert">
              {error}
            </div>
          )}

          {/* Form */}
          <form onSubmit={handleSubmit} className={styles.form}>
            {currentMode === 'signup' && step === 1 ? (
              // Signup step 1: Basic info
              <>
                <div className={styles.formRow}>
                  <div className={styles.formGroup}>
                    <label htmlFor="firstName" className={styles.label}>
                      First Name
                    </label>
                    <input
                      id="firstName"
                      type="text"
                      value={formData.firstName}
                      onChange={handleInputChange('firstName')}
                      className={styles.input}
                      required
                      aria-required="true"
                    />
                  </div>
                  <div className={styles.formGroup}>
                    <label htmlFor="lastName" className={styles.label}>
                      Last Name
                    </label>
                    <input
                      id="lastName"
                      type="text"
                      value={formData.lastName}
                      onChange={handleInputChange('lastName')}
                      className={styles.input}
                      required
                      aria-required="true"
                    />
                  </div>
                </div>
              </>
            ) : null}

            <div className={styles.formGroup}>
              <label htmlFor="email" className={styles.label}>
                Email Address
              </label>
              <input
                id="email"
                type="email"
                value={formData.email}
                onChange={handleInputChange('email')}
                className={styles.input}
                required
                aria-required="true"
                autoComplete={currentMode === 'signin' ? 'email' : 'email'}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password" className={styles.label}>
                Password
              </label>
              <input
                id="password"
                type="password"
                value={formData.password}
                onChange={handleInputChange('password')}
                className={styles.input}
                required
                aria-required="true"
                autoComplete={currentMode === 'signin' ? 'current-password' : 'new-password'}
                minLength={8}
              />
            </div>

            {currentMode === 'signup' && step === 1 && (
              <div className={styles.formGroup}>
                <label htmlFor="confirmPassword" className={styles.label}>
                  Confirm Password
                </label>
                <input
                  id="confirmPassword"
                  type="password"
                  value={formData.confirmPassword}
                  onChange={handleInputChange('confirmPassword')}
                  className={styles.input}
                  required
                  aria-required="true"
                  autoComplete="new-password"
                />
              </div>
            )}

            {currentMode === 'signup' && step === 2 && (
              // Signup step 2: Profile questions
              <div className={styles.profileQuestions}>
                <h3 className={styles.sectionTitle}>Tell us about yourself</h3>

                <div className={styles.formGroup}>
                  <label htmlFor="experienceLevel" className={styles.label}>
                    What's your experience level?
                  </label>
                  <select
                    id="experienceLevel"
                    value={profileQuestions.experienceLevel}
                    onChange={handleProfileChange('experienceLevel')}
                    className={styles.select}
                    required
                  >
                    <option value="">Select level</option>
                    <option value="beginner">Beginner - Just starting out</option>
                    <option value="intermediate">Intermediate - Some experience</option>
                    <option value="advanced">Advanced - Experienced developer</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="learningGoal" className={styles.label}>
                    What's your main learning goal?
                  </label>
                  <textarea
                    id="learningGoal"
                    value={profileQuestions.learningGoal}
                    onChange={handleProfileChange('learningGoal')}
                    className={styles.textarea}
                    rows={3}
                    placeholder="e.g., Learn web development, switch careers, etc."
                  />
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="programmingBackground" className={styles.label}>
                    Programming background
                  </label>
                  <select
                    id="programmingBackground"
                    value={profileQuestions.programmingBackground}
                    onChange={handleProfileChange('programmingBackground')}
                    className={styles.select}
                  >
                    <option value="">Select background</option>
                    <option value="none">No programming experience</option>
                    <option value="basic">Basic programming knowledge</option>
                    <option value="intermediate">Comfortable with programming</option>
                    <option value="expert">Professional developer</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label htmlFor="timeCommitment" className={styles.label}>
                    How much time can you commit per week?
                  </label>
                  <select
                    id="timeCommitment"
                    value={profileQuestions.timeCommitment}
                    onChange={handleProfileChange('timeCommitment')}
                    className={styles.select}
                  >
                    <option value="">Select time commitment</option>
                    <option value="1-3">1-3 hours</option>
                    <option value="3-5">3-5 hours</option>
                    <option value="5-10">5-10 hours</option>
                    <option value="10+">10+ hours</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label className={styles.label}>Topics you're interested in:</label>
                  <div className={styles.checkboxGroup}>
                    {['Web Development', 'Mobile Apps', 'Data Science', 'AI/ML', 'Cloud Computing', 'DevOps'].map(topic => (
                      <label key={topic} className={styles.checkboxLabel}>
                        <input
                          type="checkbox"
                          value={topic}
                          checked={profileQuestions.preferredTopics.includes(topic)}
                          onChange={handleProfileChange('preferredTopics')}
                          className={styles.checkbox}
                        />
                        <span>{topic}</span>
                      </label>
                    ))}
                  </div>
                </div>
              </div>
            )}

            {currentMode === 'signin' && (
              <div className={styles.rememberMe}>
                <label className={styles.checkboxLabel}>
                  <input type="checkbox" className={styles.checkbox} />
                  <span>Remember me</span>
                </label>
              </div>
            )}

            {/* Submit button */}
            <button
              type="submit"
              disabled={isLoading}
              className={styles.submitButton}
              aria-busy={isLoading}
            >
              {isLoading ? 'Please wait...' : currentMode === 'signin' ? 'Sign In' : step === 1 ? 'Continue' : 'Create Account'}
            </button>
          </form>

          {/* OAuth providers */}
          {providers.length > 0 && (
            <div className={styles.oauthSection}>
              <div className={styles.divider}>
                <span>OR</span>
              </div>
              <div className={styles.oauthButtons}>
                {providers.map(provider => (
                  <button
                    key={provider}
                    type="button"
                    onClick={() => handleOAuth(provider)}
                    disabled={isLoading}
                    className={styles.oauthButton}
                  >
                    <span className={styles.oauthIcon}>{getProviderIcon(provider)}</span>
                    Continue with {provider.charAt(0).toUpperCase() + provider.slice(1)}
                  </button>
                ))}
              </div>
            </div>
          )}

          {/* Mode switch */}
          <div className={styles.switchMode}>
            {currentMode === 'signin' ? (
              <>
                Don't have an account?{' '}
                <button
                  type="button"
                  onClick={() => switchMode('signup')}
                  className={styles.linkButton}
                >
                  Sign up
                </button>
              </>
            ) : (
              <>
                Already have an account?{' '}
                <button
                  type="button"
                  onClick={() => switchMode('signin')}
                  className={styles.linkButton}
                >
                  Sign in
                </button>
              </>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

function getProviderIcon(provider: string) {
  const icons = {
    google: '🔍',
    github: '🐙',
    microsoft: '🪟'
  };
  return icons[provider as keyof typeof icons] || '🔑';
}

export default ${componentName};`;
  }

  generateAuthModalStyles(styling) {
    if (styling !== 'css_modules') return '';

    return `
.overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
  padding: 20px;
}

.modal {
  background: white;
  border-radius: 12px;
  max-width: 500px;
  width: 100%;
  max-height: 90vh;
  overflow-y: auto;
  position: relative;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
}

.closeButton {
  position: absolute;
  top: 16px;
  right: 16px;
  background: none;
  border: none;
  font-size: 24px;
  cursor: pointer;
  color: #6b7280;
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 6px;
  transition: all 0.2s;
}

.closeButton:hover {
  background: #f3f4f6;
  color: #374151;
}

.closeButton:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

.content {
  padding: 32px;
}

.header {
  text-align: center;
  margin-bottom: 24px;
}

.title {
  font-size: 24px;
  font-weight: 600;
  color: #111827;
  margin: 0 0 8px 0;
}

.subtitle {
  font-size: 14px;
  color: #6b7280;
  margin: 0;
}

.errorMessage {
  background: #fef2f2;
  border: 1px solid #fecaca;
  color: #dc2626;
  padding: 12px;
  border-radius: 8px;
  font-size: 14px;
  margin-bottom: 20px;
}

.form {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.formRow {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 12px;
}

.formGroup {
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.label {
  font-size: 14px;
  font-weight: 500;
  color: #374151;
}

.input {
  padding: 10px 12px;
  border: 1px solid #d1d5db;
  border-radius: 6px;
  font-size: 14px;
  transition: border-color 0.2s;
}

.input:focus {
  outline: none;
  border-color: #3b82f6;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
}

.select {
  padding: 10px 12px;
  border: 1px solid #d1d5db;
  border-radius: 6px;
  font-size: 14px;
  background: white;
  cursor: pointer;
}

.select:focus {
  outline: none;
  border-color: #3b82f6;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
}

.textarea {
  padding: 10px 12px;
  border: 1px solid #d1d5db;
  border-radius: 6px;
  font-size: 14px;
  font-family: inherit;
  resize: vertical;
  min-height: 80px;
}

.textarea:focus {
  outline: none;
  border-color: #3b82f6;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
}

.checkboxGroup {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 8px;
}

.checkboxLabel {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
  cursor: pointer;
}

.checkbox {
  width: 16px;
  height: 16px;
  accent-color: #3b82f6;
}

.rememberMe {
  display: flex;
  justify-content: flex-start;
  margin: 8px 0;
}

.submitButton {
  background: #3b82f6;
  color: white;
  border: none;
  padding: 12px 24px;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: background 0.2s;
  margin-top: 8px;
  min-height: 44px;
}

.submitButton:hover:not(:disabled) {
  background: #2563eb;
}

.submitButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.submitButton:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
}

.oauthSection {
  margin: 24px 0;
}

.divider {
  display: flex;
  align-items: center;
  margin: 20px 0;
  color: #9ca3af;
  font-size: 14px;
}

.divider::before,
.divider::after {
  content: '';
  flex: 1;
  height: 1px;
  background: #e5e7eb;
}

.divider span {
  padding: 0 16px;
}

.oauthButtons {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.oauthButton {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  padding: 10px 16px;
  border: 1px solid #d1d5db;
  background: white;
  border-radius: 6px;
  font-size: 14px;
  cursor: pointer;
  transition: all 0.2s;
  min-height: 44px;
}

.oauthButton:hover:not(:disabled) {
  background: #f9fafb;
  border-color: #9ca3af;
}

.oauthButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.oauthIcon {
  font-size: 18px;
}

.switchMode {
  text-align: center;
  font-size: 14px;
  color: #6b7280;
  margin-top: 20px;
}

.linkButton {
  background: none;
  border: none;
  color: #3b82f6;
  font-weight: 500;
  cursor: pointer;
  text-decoration: underline;
  padding: 0;
  font-size: inherit;
}

.linkButton:hover {
  color: #2563eb;
}

.linkButton:focus {
  outline: 2px solid #3b82f6;
  outline-offset: 2px;
  border-radius: 2px;
}

.profileQuestions {
  display: flex;
  flex-direction: column;
  gap: 16px;
  margin-top: 20px;
  padding-top: 20px;
  border-top: 1px solid #e5e7eb;
}

.sectionTitle {
  font-size: 16px;
  font-weight: 600;
  color: #111827;
  margin: 0;
}

/* Dark theme */
[data-theme='dark'] .modal {
  background: #1f2937;
  color: #f3f4f6;
}

[data-theme='dark'] .title {
  color: #f9fafb;
}

[data-theme='dark'] .subtitle,
[data-theme='dark'] .switchMode {
  color: #9ca3af;
}

[data-theme='dark'] .input,
[data-theme='dark'] .select,
[data-theme='dark'] .textarea {
  background: #374151;
  border-color: #4b5563;
  color: #f3f4f6;
}

[data-theme='dark'] .input:focus,
[data-theme='dark'] .select:focus,
[data-theme='dark'] .textarea:focus {
  border-color: #60a5fa;
  box-shadow: 0 0 0 3px rgba(96, 165, 250, 0.1);
}

[data-theme='dark'] .oauthButton {
  background: #374151;
  border-color: #4b5563;
  color: #f3f4f6;
}

[data-theme='dark'] .oauthButton:hover:not(:disabled) {
  background: #4b5563;
}

[data-theme='dark'] .divider span {
  color: #6b7280;
}

[data-theme='dark'] .divider::before,
[data-theme='dark'] .divider::after {
  background: #374151;
}

/* Mobile responsive */
@media (max-width: 480px) {
  .content {
    padding: 24px 16px;
  }

  .formRow {
    grid-template-columns: 1fr;
  }

  .oauthButtons {
    flex-direction: column;
  }

  .checkboxGroup {
    grid-template-columns: 1fr;
  }
}

/* Animation for modal appearance */
.modal {
  animation: modalAppear 0.3s ease;
}

@keyframes modalAppear {
  from {
    opacity: 0;
    transform: scale(0.9);
  }
  to {
    opacity: 1;
    transform: scale(1);
  }
}

/* High contrast mode */
@media (prefers-contrast: high) {
  .modal {
    border: 2px solid currentColor;
  }

  .input,
  .select,
  .textarea {
    border: 2px solid currentColor;
  }
}

/* Reduced motion */
@media (prefers-reduced-motion: reduce) {
  .modal {
    animation: none;
  }

  .submitButton,
  .oauthButton,
  .closeButton {
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
  const mockOnSuccess = jest.fn();
  const mockOnClose = jest.fn();

  beforeEach(() => {
    mockFetch.mockClear();
    mockOnSuccess.mockClear();
    mockOnClose.mockClear();
  });

  it('renders signin form by default', () => {
    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    expect(screen.getByText('Sign In')).toBeInTheDocument();
    expect(screen.getByLabelText('Email Address')).toBeInTheDocument();
    expect(screen.getByLabelText('Password')).toBeInTheDocument();
    expect(screen.queryByLabelText('First Name')).not.toBeInTheDocument();
  });

  it('renders signup form when mode is signup', () => {
    render(
      <${componentName}
        mode="signup"
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    expect(screen.getByText('Create Account')).toBeInTheDocument();
    expect(screen.getByLabelText('First Name')).toBeInTheDocument();
    expect(screen.getByLabelText('Last Name')).toBeInTheDocument();
    expect(screen.getByLabelText('Confirm Password')).toBeInTheDocument();
  });

  it('switches between signin and signup modes', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    // Click sign up link
    const signUpLink = screen.getByRole('button', { name: /sign up/i });
    await user.click(signUpLink);

    expect(screen.getByText('Create Account')).toBeInTheDocument();

    // Click sign in link
    const signInLink = screen.getByRole('button', { name: /sign in/i });
    await user.click(signInLink);

    expect(screen.getByText('Sign In')).toBeInTheDocument();
  });

  it('closes modal when close button is clicked', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const closeButton = screen.getByRole('button', { name: /close authentication modal/i });
    await user.click(closeButton);

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('closes modal when escape key is pressed', async () => {
    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    fireEvent.keyDown(document, { key: 'Escape' });

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('validates email format', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        mode="signup"
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const emailInput = screen.getByLabelText('Email Address');
    const passwordInput = screen.getByLabelText('Password');
    const submitButton = screen.getByRole('button', { name: /continue/i });

    await user.type(emailInput, 'invalid-email');
    await user.type(passwordInput, 'password123');
    await user.click(submitButton);

    expect(screen.getByText('Please enter a valid email address')).toBeInTheDocument();
  });

  it('validates password length', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        mode="signup"
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const emailInput = screen.getByLabelText('Email Address');
    const passwordInput = screen.getByLabelText('Password');
    const submitButton = screen.getByRole('button', { name: /continue/i });

    await user.type(emailInput, 'test@example.com');
    await user.type(passwordInput, '123');
    await user.click(submitButton);

    expect(screen.getByText('Password must be at least 8 characters long')).toBeInTheDocument();
  });

  it('validates password confirmation', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        mode="signup"
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const emailInput = screen.getByLabelText('Email Address');
    const passwordInput = screen.getByLabelText('Password');
    const confirmPasswordInput = screen.getByLabelText('Confirm Password');
    const submitButton = screen.getByRole('button', { name: /continue/i });

    await user.type(emailInput, 'test@example.com');
    await user.type(passwordInput, 'password123');
    await user.type(confirmPasswordInput, 'differentpassword');
    await user.click(submitButton);

    expect(screen.getByText('Passwords do not match')).toBeInTheDocument();
  });

  it('moves to profile questions step on valid signup form', async () => {
    const user = userEvent.setup();
    render(
      <${componentName}
        mode="signup"
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const firstNameInput = screen.getByLabelText('First Name');
    const lastNameInput = screen.getByLabelText('Last Name');
    const emailInput = screen.getByLabelText('Email Address');
    const passwordInput = screen.getByLabelText('Password');
    const confirmPasswordInput = screen.getByLabelText('Confirm Password');
    const submitButton = screen.getByRole('button', { name: /continue/i });

    await user.type(firstNameInput, 'John');
    await user.type(lastNameInput, 'Doe');
    await user.type(emailInput, 'john@example.com');
    await user.type(passwordInput, 'password123');
    await user.type(confirmPasswordInput, 'password123');
    await user.click(submitButton);

    expect(screen.getByText('Tell us about yourself')).toBeInTheDocument();
    expect(screen.getByLabelText("What's your experience level?")).toBeInTheDocument();
  });

  it('submits signin form successfully', async () => {
    const user = userEvent.setup();
    const mockUser = { id: '1', email: 'test@example.com', name: 'Test User' };
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ user: mockUser })
    });

    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const emailInput = screen.getByLabelText('Email Address');
    const passwordInput = screen.getByLabelText('Password');
    const submitButton = screen.getByRole('button', { name: /sign in/i });

    await user.type(emailInput, 'test@example.com');
    await user.type(passwordInput, 'password123');
    await user.click(submitButton);

    await waitFor(() => {
      expect(mockFetch).toHaveBeenCalledWith('/api/auth/signin', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: 'test@example.com',
          password: 'password123'
        })
      });
    });

    await waitFor(() => {
      expect(mockOnSuccess).toHaveBeenCalledWith(mockUser);
    });

    expect(mockOnClose).toHaveBeenCalled();
  });

  it('displays OAuth provider buttons', () => {
    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
        providers={['google', 'github']}
      />
    );

    expect(screen.getByText('Continue with Google')).toBeInTheDocument();
    expect(screen.getByText('Continue with GitHub')).toBeInTheDocument();
  });

  it('traps focus within modal', () => {
    render(
      <${componentName}
        onSuccess={mockOnSuccess}
        onClose={mockOnClose}
      />
    );

    const modal = screen.getByRole('dialog');
    expect(modal).toHaveFocus();
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
    mode: {
      control: 'select',
      options: ['signin', 'signup'],
    },
    providers: {
      control: 'check',
      options: ['google', 'github', 'microsoft'],
    },
  },
};

export default meta;
type Story = StoryObj<typeof meta>;

export const Signin: Story = {
  args: {
    mode: 'signin',
    onSuccess: (user) => console.log('Signed in:', user),
    onClose: () => console.log('Modal closed'),
  },
};

export const SigninWithOAuth: Story = {
  args: {
    mode: 'signin',
    providers: ['google', 'github', 'microsoft'],
    onSuccess: (user) => console.log('Signed in:', user),
    onClose: () => console.log('Modal closed'),
  },
};

export const Signup: Story = {
  args: {
    mode: 'signup',
    onSuccess: (user) => console.log('Signed up:', user),
    onClose: () => console.log('Modal closed'),
  },
};

export const SignupWithOAuth: Story = {
  args: {
    mode: 'signup',
    providers: ['google', 'github'],
    onSuccess: (user) => console.log('Signed up:', user),
    onClose: () => console.log('Modal closed'),
  },
};`;
  }
}

module.exports = AuthModalGenerator;