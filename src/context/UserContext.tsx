import React, { createContext, useContext, ReactNode } from 'react';

export interface UserProfile {
  experienceLevel: 'beginner' | 'intermediate' | 'advanced';
  rosFamiliarity: 'novice' | 'intermediate' | 'expert';
  hardwareAccess: boolean;
}

interface UserContextType {
  userProfile: UserProfile | null;
  setUserProfile: (profile: UserProfile | null) => void;
}

const defaultUserContext: UserContextType = {
  userProfile: null,
  setUserProfile: () => {},
};

export const UserContext = createContext<UserContextType>(defaultUserContext);

interface UserProviderProps {
  children: ReactNode;
}

export function UserProvider({ children }: UserProviderProps) {
  const [userProfile, setUserProfile] = React.useState<UserProfile | null>(() => {
    // Load from localStorage if available
    if (typeof window !== 'undefined') {
      const saved = localStorage.getItem('userProfile');
      return saved ? JSON.parse(saved) : null;
    }
    return null;
  });

  React.useEffect(() => {
    // Save to localStorage when profile changes
    if (userProfile && typeof window !== 'undefined') {
      localStorage.setItem('userProfile', JSON.stringify(userProfile));
    }
  }, [userProfile]);

  return (
    <UserContext.Provider value={{ userProfile, setUserProfile }}>
      {children}
    </UserContext.Provider>
  );
}

export function useUserContext() {
  return useContext(UserContext);
}