/**
 * Logout button component.
 * Handles user logout with confirmation.
 */

import React, { useCallback, useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './AuthForms.module.css';

interface LogoutButtonProps {
  className?: string;
  showIcon?: boolean;
}

export function LogoutButton({ className, showIcon = true }: LogoutButtonProps): JSX.Element {
  const { logout, isLoading } = useAuth();
  const [isLoggingOut, setIsLoggingOut] = useState(false);

  const handleLogout = useCallback(async () => {
    setIsLoggingOut(true);
    try {
      await logout();
    } finally {
      setIsLoggingOut(false);
    }
  }, [logout]);

  const buttonClasses = className ? `${styles.logoutButton} ${className}` : styles.logoutButton;

  return (
    <button
      onClick={handleLogout}
      disabled={isLoading || isLoggingOut}
      className={buttonClasses}
      type="button"
    >
      {showIcon && (
        <span aria-hidden="true" role="img">
          🚪
        </span>
      )}
      {isLoggingOut ? 'Logging out...' : 'Log Out'}
    </button>
  );
}

export default LogoutButton;
