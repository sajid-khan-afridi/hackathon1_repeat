/**
 * Modal wrapper for authentication forms.
 * Provides accessible modal dialog for login/signup.
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { LoginForm } from './LoginForm';
import { SignupForm } from './SignupForm';
import styles from './AuthForms.module.css';

type AuthMode = 'login' | 'signup';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  initialMode?: AuthMode;
}

export function AuthModal({
  isOpen,
  onClose,
  initialMode = 'login',
}: AuthModalProps): JSX.Element | null {
  const [mode, setMode] = useState<AuthMode>(initialMode);
  const modalRef = useRef<HTMLDivElement>(null);
  const previousActiveElement = useRef<HTMLElement | null>(null);

  // Reset mode when modal opens
  useEffect(() => {
    if (isOpen) {
      setMode(initialMode);
      previousActiveElement.current = document.activeElement as HTMLElement;
    }
  }, [isOpen, initialMode]);

  // Focus trap and keyboard handling
  useEffect(() => {
    if (!isOpen) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
        return;
      }

      // Focus trap
      if (e.key === 'Tab' && modalRef.current) {
        const focusableElements = modalRef.current.querySelectorAll<HTMLElement>(
          'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
        );
        const firstElement = focusableElements[0];
        const lastElement = focusableElements[focusableElements.length - 1];

        if (e.shiftKey && document.activeElement === firstElement) {
          e.preventDefault();
          lastElement?.focus();
        } else if (!e.shiftKey && document.activeElement === lastElement) {
          e.preventDefault();
          firstElement?.focus();
        }
      }
    };

    document.addEventListener('keydown', handleKeyDown);

    // Focus first focusable element
    const focusableElements = modalRef.current?.querySelectorAll<HTMLElement>(
      'button, [href], input, select, textarea, [tabindex]:not([tabindex="-1"])'
    );
    focusableElements?.[0]?.focus();

    return () => {
      document.removeEventListener('keydown', handleKeyDown);
      // Restore focus when modal closes
      previousActiveElement.current?.focus();
    };
  }, [isOpen, onClose]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen]);

  const handleSuccess = useCallback(() => {
    onClose();
  }, [onClose]);

  const switchToLogin = useCallback(() => {
    setMode('login');
  }, []);

  const switchToSignup = useCallback(() => {
    setMode('signup');
  }, []);

  const handleBackdropClick = useCallback(
    (e: React.MouseEvent) => {
      if (e.target === e.currentTarget) {
        onClose();
      }
    },
    [onClose]
  );

  if (!isOpen) return null;

  return (
    <div
      className={styles.modal}
      role="dialog"
      aria-modal="true"
      aria-labelledby="auth-modal-title"
      ref={modalRef}
    >
      <div className={styles.modalBackdrop} onClick={handleBackdropClick} />
      <div className={styles.modalContent}>
        <button
          className={styles.modalClose}
          onClick={onClose}
          aria-label="Close"
          type="button"
        >
          ×
        </button>
        <h2 id="auth-modal-title" className="visually-hidden">
          {mode === 'login' ? 'Log In' : 'Create Account'}
        </h2>
        {mode === 'login' ? (
          <LoginForm onSuccess={handleSuccess} onSignupClick={switchToSignup} />
        ) : (
          <SignupForm onSuccess={handleSuccess} onLoginClick={switchToLogin} />
        )}
      </div>
    </div>
  );
}

export default AuthModal;
