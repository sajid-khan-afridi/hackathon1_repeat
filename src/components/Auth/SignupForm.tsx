/**
 * Signup form component with email/password registration.
 * Implements FR-001, FR-002, FR-003, FR-004 (signup, validation, password requirements).
 */

import React, { useState, FormEvent, useCallback, useMemo } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { AuthApiError } from '../../services/authApi';
import { GoogleLoginButton } from './GoogleLoginButton';
import { GitHubLoginButton } from './GitHubLoginButton';
import styles from './AuthForms.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
  onLoginClick?: () => void;
}

interface PasswordRequirement {
  label: string;
  test: (password: string) => boolean;
}

const PASSWORD_REQUIREMENTS: PasswordRequirement[] = [
  {
    label: 'At least 8 characters',
    test: (p) => p.length >= 8,
  },
  {
    label: 'At least 1 uppercase letter',
    test: (p) => /[A-Z]/.test(p),
  },
  {
    label: 'At least 1 number',
    test: (p) => /[0-9]/.test(p),
  },
];

export function SignupForm({ onSuccess, onLoginClick }: SignupFormProps): JSX.Element {
  const { signup, isLoading } = useAuth();

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [fieldErrors, setFieldErrors] = useState<{
    email?: string;
    password?: string;
    confirmPassword?: string;
  }>({});
  const [showPasswordRequirements, setShowPasswordRequirements] = useState(false);

  // Check password requirements
  const passwordChecks = useMemo(
    () =>
      PASSWORD_REQUIREMENTS.map((req) => ({
        ...req,
        passed: req.test(password),
      })),
    [password]
  );

  const allPasswordRequirementsMet = passwordChecks.every((req) => req.passed);

  // Validate email format
  const validateEmail = (value: string): string | undefined => {
    if (!value) {
      return 'Email is required';
    }
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(value)) {
      return 'Please enter a valid email address';
    }
    return undefined;
  };

  // Validate password
  const validatePassword = (value: string): string | undefined => {
    if (!value) {
      return 'Password is required';
    }
    if (!allPasswordRequirementsMet) {
      return 'Password does not meet all requirements';
    }
    return undefined;
  };

  // Validate confirm password
  const validateConfirmPassword = (value: string): string | undefined => {
    if (!value) {
      return 'Please confirm your password';
    }
    if (value !== password) {
      return 'Passwords do not match';
    }
    return undefined;
  };

  // Handle form submission
  const handleSubmit = useCallback(
    async (e: FormEvent) => {
      e.preventDefault();
      setError(null);

      // Validate all fields
      const emailError = validateEmail(email);
      const passwordError = validatePassword(password);
      const confirmError = validateConfirmPassword(confirmPassword);

      if (emailError || passwordError || confirmError) {
        setFieldErrors({
          email: emailError,
          password: passwordError,
          confirmPassword: confirmError,
        });
        return;
      }

      setFieldErrors({});

      try {
        await signup({ email, password });
        onSuccess?.();
      } catch (err) {
        if (err instanceof AuthApiError) {
          if (err.code === 'EMAIL_EXISTS') {
            setError('This email is already registered. Please log in instead.');
          } else if (err.details) {
            // Show field-specific validation errors
            const newFieldErrors: typeof fieldErrors = {};
            err.details.forEach((detail) => {
              if (detail.field === 'email') newFieldErrors.email = detail.message;
              if (detail.field === 'password') newFieldErrors.password = detail.message;
            });
            setFieldErrors(newFieldErrors);
          } else {
            setError(err.message || 'An error occurred during signup.');
          }
        } else {
          setError('An error occurred. Please try again.');
        }
      }
    },
    [email, password, confirmPassword, signup, onSuccess, allPasswordRequirementsMet]
  );

  // Handle field blur for validation
  const handleEmailBlur = () => {
    const error = validateEmail(email);
    setFieldErrors((prev) => ({ ...prev, email: error }));
  };

  const handleConfirmPasswordBlur = () => {
    const error = validateConfirmPassword(confirmPassword);
    setFieldErrors((prev) => ({ ...prev, confirmPassword: error }));
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm} noValidate>
      <h2 className={styles.formTitle}>Create Account</h2>

      {error && (
        <div className={styles.errorMessage} role="alert">
          {error}
        </div>
      )}

      <div className={styles.formGroup}>
        <label htmlFor="signup-email" className={styles.label}>
          Email
        </label>
        <input
          id="signup-email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          onBlur={handleEmailBlur}
          className={`${styles.input} ${fieldErrors.email ? styles.inputError : ''}`}
          placeholder="you@example.com"
          autoComplete="email"
          aria-describedby={fieldErrors.email ? 'signup-email-error' : undefined}
          aria-invalid={!!fieldErrors.email}
          disabled={isLoading}
          required
        />
        {fieldErrors.email && (
          <span id="signup-email-error" className={styles.fieldError} role="alert">
            {fieldErrors.email}
          </span>
        )}
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="signup-password" className={styles.label}>
          Password
        </label>
        <input
          id="signup-password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          onFocus={() => setShowPasswordRequirements(true)}
          onBlur={() => setShowPasswordRequirements(false)}
          className={`${styles.input} ${fieldErrors.password ? styles.inputError : ''}`}
          placeholder="Create a strong password"
          autoComplete="new-password"
          aria-describedby="signup-password-requirements"
          aria-invalid={!!fieldErrors.password}
          disabled={isLoading}
          required
        />
        {(showPasswordRequirements || password) && (
          <ul
            id="signup-password-requirements"
            className={styles.passwordRequirements}
            aria-label="Password requirements"
          >
            {passwordChecks.map((req, index) => (
              <li key={index} className={req.passed ? styles.requirementMet : styles.requirement}>
                <span className={styles.requirementIcon} aria-hidden="true">
                  {req.passed ? '✓' : '○'}
                </span>
                <span className={req.passed ? styles.requirementTextMet : ''}>{req.label}</span>
              </li>
            ))}
          </ul>
        )}
        {fieldErrors.password && (
          <span id="signup-password-error" className={styles.fieldError} role="alert">
            {fieldErrors.password}
          </span>
        )}
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="signup-confirm-password" className={styles.label}>
          Confirm Password
        </label>
        <input
          id="signup-confirm-password"
          type="password"
          value={confirmPassword}
          onChange={(e) => setConfirmPassword(e.target.value)}
          onBlur={handleConfirmPasswordBlur}
          className={`${styles.input} ${fieldErrors.confirmPassword ? styles.inputError : ''}`}
          placeholder="Confirm your password"
          autoComplete="new-password"
          aria-describedby={fieldErrors.confirmPassword ? 'signup-confirm-error' : undefined}
          aria-invalid={!!fieldErrors.confirmPassword}
          disabled={isLoading}
          required
        />
        {fieldErrors.confirmPassword && (
          <span id="signup-confirm-error" className={styles.fieldError} role="alert">
            {fieldErrors.confirmPassword}
          </span>
        )}
      </div>

      <button type="submit" className={styles.submitButton} disabled={isLoading}>
        {isLoading ? 'Creating account...' : 'Create Account'}
      </button>

      <div className={styles.divider}>or</div>

      <GoogleLoginButton label="Sign up with Google" disabled={isLoading} />

      <div style={{ marginTop: '0.75rem' }}>
        <GitHubLoginButton label="Sign up with GitHub" disabled={isLoading} />
      </div>

      {onLoginClick && (
        <p className={styles.switchText}>
          Already have an account?{' '}
          <button type="button" onClick={onLoginClick} className={styles.linkButton}>
            Log in
          </button>
        </p>
      )}
    </form>
  );
}

export default SignupForm;
