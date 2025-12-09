"use client";

import { useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import { authClient } from "@/lib/auth";
import type { UserProfile } from "@/db/schema";

export function useAuth() {
  const [user, setUser] = useState<any>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const router = useRouter();

  // Fetch user session
  const fetchSession = async () => {
    try {
      const session = await authClient.getSession();
      if (session.data?.user) {
        setUser(session.data.user);
        setIsAuthenticated(true);
        // Fetch user profile
        await fetchProfile();
      } else {
        setUser(null);
        setProfile(null);
        setIsAuthenticated(false);
      }
    } catch (error) {
      console.error("Error fetching session:", error);
      setUser(null);
      setProfile(null);
      setIsAuthenticated(false);
    } finally {
      setIsLoading(false);
    }
  };

  // Fetch user profile
  const fetchProfile = async () => {
    try {
      const response = await fetch("/api/auth/profile");
      if (response.ok) {
        const profileData = await response.json();
        setProfile(profileData);
      }
    } catch (error) {
      console.error("Error fetching profile:", error);
    }
  };

  // Sign out
  const signOut = async () => {
    try {
      await authClient.signOut();
      setUser(null);
      setProfile(null);
      setIsAuthenticated(false);
      router.push("/auth/signin");
    } catch (error) {
      console.error("Error signing out:", error);
    }
  };

  // Update profile
  const updateProfile = async (updates: Partial<UserProfile>) => {
    try {
      const response = await fetch("/api/auth/profile", {
        method: "PUT",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(updates),
      });

      if (response.ok) {
        const updatedProfile = await response.json();
        setProfile(updatedProfile);
        return true;
      }
      return false;
    } catch (error) {
      console.error("Error updating profile:", error);
      return false;
    }
  };

  // Check if profile is complete
  const hasCompleteProfile = profile ? Object.keys(profile).length > 0 : false;

  useEffect(() => {
    fetchSession();
  }, []);

  // Refresh session periodically
  useEffect(() => {
    if (isAuthenticated) {
      const interval = setInterval(fetchSession, 5 * 60 * 1000); // Every 5 minutes
      return () => clearInterval(interval);
    }
  }, [isAuthenticated]);

  return {
    user,
    profile,
    isLoading,
    isAuthenticated,
    hasCompleteProfile,
    signOut,
    updateProfile,
    refetch: fetchSession,
  };
}

// Hook for protected routes
export function useRequireAuth() {
  const { isAuthenticated, isLoading, user } = useAuth();
  const router = useRouter();

  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      router.push("/auth/signin");
    }
  }, [isAuthenticated, isLoading, router]);

  return { isAuthenticated, isLoading, user };
}

// Hook that requires complete profile
export function useRequireProfile() {
  const { isAuthenticated, hasCompleteProfile, isLoading, user } = useAuth();
  const router = useRouter();

  useEffect(() => {
    if (!isLoading) {
      if (!isAuthenticated) {
        router.push("/auth/signin");
      } else if (!hasCompleteProfile) {
        router.push("/auth/profile-complete");
      }
    }
  }, [isAuthenticated, hasCompleteProfile, isLoading, router]);

  return { isAuthenticated, hasCompleteProfile, isLoading, user };
}

// Hook for role-based access (if needed in future)
export function useRequireRole(requiredRoles: string[]) {
  const { isAuthenticated, user, isLoading } = useAuth();
  const router = useRouter();

  useEffect(() => {
    if (!isLoading) {
      if (!isAuthenticated) {
        router.push("/auth/signin");
        return;
      }

      // Check user roles (assuming role field exists)
      const userRoles = user?.roles || [];
      const hasRequiredRole = requiredRoles.some(role => userRoles.includes(role));

      if (!hasRequiredRole) {
        router.push("/unauthorized");
      }
    }
  }, [isAuthenticated, user, isLoading, requiredRoles, router]);

  return { isAuthenticated, user, isLoading };
}