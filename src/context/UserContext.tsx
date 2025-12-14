import React, { createContext, useContext, useCallback, ReactNode } from 'react';

export interface UserProfile {
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity: 'novice' | 'intermediate' | 'expert';
  hardwareAccess: boolean;
}

interface UserContextType {
  userProfile: UserProfile | null;
  setUserProfile: (profile: UserProfile | null) => void;
}

const STORAGE_KEY = 'userProfile';

const defaultUserContext: UserContextType = {
  userProfile: null,
  setUserProfile: () => {},
};

export const UserContext = createContext<UserContextType>(defaultUserContext);

interface UserProviderProps {
  children: ReactNode;
}

/**
 * Safely read from localStorage with error handling
 */
function loadFromStorage(): UserProfile | null {
  if (typeof window === 'undefined') {
    return null;
  }
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      const parsed = JSON.parse(saved);
      // Validate the shape of the data
      if (
        parsed &&
        typeof parsed.experienceLevel === 'string' &&
        typeof parsed.rosFamiliarity === 'string' &&
        typeof parsed.hardwareAccess === 'boolean'
      ) {
        return parsed as UserProfile;
      }
    }
  } catch (error) {
    console.warn('Failed to load user profile from localStorage:', error);
  }
  return null;
}

/**
 * Safely write to localStorage with error handling
 */
function saveToStorage(profile: UserProfile | null): void {
  if (typeof window === 'undefined') {
    return;
  }
  try {
    if (profile) {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(profile));
    } else {
      localStorage.removeItem(STORAGE_KEY);
    }
  } catch (error) {
    console.warn('Failed to save user profile to localStorage:', error);
  }
}

export function UserProvider({ children }: UserProviderProps) {
  const [userProfile, setUserProfileState] = React.useState<UserProfile | null>(loadFromStorage);

  // Memoized setter that also persists to localStorage
  const setUserProfile = useCallback((profile: UserProfile | null) => {
    setUserProfileState(profile);
    saveToStorage(profile);
  }, []);

  return (
    <UserContext.Provider value={{ userProfile, setUserProfile }}>
      {children}
    </UserContext.Provider>
  );
}

export function useUserContext() {
  return useContext(UserContext);
}