/**
 * Login form component with email/password authentication.
 * Implements FR-006, FR-030, FR-031 (form validation, error messages).
 */

import React, { useState, FormEvent, useCallback } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { AuthApiError } from '../../services/authApi';
import { GoogleLoginButton } from './GoogleLoginButton';
import { GitHubLoginButton } from './GitHubLoginButton';
import styles from './AuthForms.module.css';

interface LoginFormProps {
  onSuccess?: () => void;
  onSignupClick?: () => void;
}

export function LoginForm({
  onSuccess,
  onSignupClick,
}: LoginFormProps): JSX.Element {
  const { login, isLoading } = useAuth();

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [fieldErrors, setFieldErrors] = useState<{
    email?: string;
    password?: string;
  }>({});

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
    return undefined;
  };

  // Handle form submission
  const handleSubmit = useCallback(
    async (e: FormEvent) => {
      e.preventDefault();
      setError(null);

      // Validate fields
      const emailError = validateEmail(email);
      const passwordError = validatePassword(password);

      if (emailError || passwordError) {
        setFieldErrors({ email: emailError, password: passwordError });
        return;
      }

      setFieldErrors({});

      try {
        await login({ email, password });
        onSuccess?.();
      } catch (err) {
        if (err instanceof AuthApiError) {
          if (err.code === 'ACCOUNT_LOCKED') {
            const retryMinutes = err.retryAfter
              ? Math.ceil(err.retryAfter / 60)
              : 15;
            setError(
              `Account locked due to too many failed attempts. Please try again in ${retryMinutes} minutes.`
            );
          } else if (err.code === 'NO_PASSWORD') {
            setError('Please use Google login for this account.');
          } else {
            setError('Invalid email or password. Please try again.');
          }
        } else {
          setError('An error occurred. Please try again.');
        }
      }
    },
    [email, password, login, onSuccess]
  );

  // Handle field blur for validation
  const handleEmailBlur = () => {
    const error = validateEmail(email);
    setFieldErrors((prev) => ({ ...prev, email: error }));
  };

  const handlePasswordBlur = () => {
    const error = validatePassword(password);
    setFieldErrors((prev) => ({ ...prev, password: error }));
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm} noValidate>
      <h2 className={styles.formTitle}>Log In</h2>

      {error && (
        <div className={styles.errorMessage} role="alert">
          {error}
        </div>
      )}

      <div className={styles.formGroup}>
        <label htmlFor="login-email" className={styles.label}>
          Email
        </label>
        <input
          id="login-email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          onBlur={handleEmailBlur}
          className={`${styles.input} ${fieldErrors.email ? styles.inputError : ''}`}
          placeholder="you@example.com"
          autoComplete="email"
          aria-describedby={fieldErrors.email ? 'login-email-error' : undefined}
          aria-invalid={!!fieldErrors.email}
          disabled={isLoading}
          required
        />
        {fieldErrors.email && (
          <span id="login-email-error" className={styles.fieldError} role="alert">
            {fieldErrors.email}
          </span>
        )}
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="login-password" className={styles.label}>
          Password
        </label>
        <input
          id="login-password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          onBlur={handlePasswordBlur}
          className={`${styles.input} ${fieldErrors.password ? styles.inputError : ''}`}
          placeholder="Enter your password"
          autoComplete="current-password"
          aria-describedby={
            fieldErrors.password ? 'login-password-error' : undefined
          }
          aria-invalid={!!fieldErrors.password}
          disabled={isLoading}
          required
        />
        {fieldErrors.password && (
          <span
            id="login-password-error"
            className={styles.fieldError}
            role="alert"
          >
            {fieldErrors.password}
          </span>
        )}
      </div>

      <button
        type="submit"
        className={styles.submitButton}
        disabled={isLoading}
      >
        {isLoading ? 'Logging in...' : 'Log In'}
      </button>

      <div className={styles.divider}>or</div>

      <GoogleLoginButton disabled={isLoading} />

      <div style={{ marginTop: '0.75rem' }}>
        <GitHubLoginButton disabled={isLoading} />
      </div>

      {onSignupClick && (
        <p className={styles.switchText}>
          Don't have an account?{' '}
          <button
            type="button"
            onClick={onSignupClick}
            className={styles.linkButton}
          >
            Sign up
          </button>
        </p>
      )}
    </form>
  );
}

export default LoginForm;
