import React, { createContext, useContext, useCallback, useEffect, ReactNode } from 'react';
import { useAuthContext } from './AuthContext';
import type { UserProfile as AuthUserProfile, ROSFamiliarity, HardwareAccess } from '../types/auth';

export interface UserProfile {
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity: 'novice' | 'intermediate' | 'expert';
  hardwareAccess: boolean;
  preferredLanguage?: 'python' | 'cpp' | 'both';
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

/**
 * Maps AuthContext ROSFamiliarity to UserContext rosFamiliarity
 */
function mapRosFamiliarity(ros: ROSFamiliarity | null): 'novice' | 'intermediate' | 'expert' {
  switch (ros) {
    case 'none':
      return 'novice';
    case 'basic':
      return 'intermediate';
    case 'proficient':
      return 'expert';
    default:
      return 'novice';
  }
}

/**
 * Maps AuthContext HardwareAccess to UserContext hardwareAccess (boolean)
 */
function mapHardwareAccess(hardware: HardwareAccess | null): boolean {
  // simulation_only means no physical hardware
  return hardware !== null && hardware !== 'simulation_only';
}

/**
 * Converts AuthContext profile to UserContext profile format
 */
function mapAuthProfileToUserProfile(authProfile: AuthUserProfile | null): UserProfile | null {
  if (!authProfile || !authProfile.experienceLevel) {
    return null;
  }

  return {
    experienceLevel: authProfile.experienceLevel,
    rosFamiliarity: mapRosFamiliarity(authProfile.rosFamiliarity),
    hardwareAccess: mapHardwareAccess(authProfile.hardwareAccess),
    preferredLanguage: authProfile.preferredLanguage || undefined,
  };
}

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
        typeof parsed.hardwareAccess === 'boolean' &&
        (parsed.preferredLanguage === undefined || typeof parsed.preferredLanguage === 'string')
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

  // Get auth profile to sync with
  const { profile: authProfile, isAuthenticated } = useAuthContext();

  // Memoized setter that also persists to localStorage
  const setUserProfile = useCallback((profile: UserProfile | null) => {
    setUserProfileState(profile);
    saveToStorage(profile);
  }, []);

  // Sync UserContext with AuthContext profile when it changes
  // This ensures PersonalizedSection sees updates from ProfileSettings
  useEffect(() => {
    if (isAuthenticated && authProfile) {
      const mappedProfile = mapAuthProfileToUserProfile(authProfile);
      if (mappedProfile) {
        setUserProfileState(mappedProfile);
        saveToStorage(mappedProfile);
      }
    } else if (!isAuthenticated) {
      // Clear profile on logout
      setUserProfileState(null);
      saveToStorage(null);
    }
  }, [authProfile, isAuthenticated]);

  return (
    <UserContext.Provider value={{ userProfile, setUserProfile }}>{children}</UserContext.Provider>
  );
}

export function useUserContext() {
  return useContext(UserContext);
}
