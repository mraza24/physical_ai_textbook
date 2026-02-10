import React, { useEffect } from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import { useAuth } from '@site/src/hooks/useAuth';

/**
 * Wrapper for Docusaurus Navbar with session debug logging
 *
 * This adds console logging to track auth state for debugging
 * session persistence issues.
 */
export default function Navbar(props) {
  const { user, profile, isAuthenticated, isLoading } = useAuth();

  // Log session state on mount and when auth changes
  useEffect(() => {
    const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;
    const storedUser = typeof window !== 'undefined' ? localStorage.getItem('user') : null;
    const storedProfile = typeof window !== 'undefined' ? localStorage.getItem('profile') : null;

    console.log('[Navbar] User Session:', {
      isAuthenticated,
      isLoading,
      user: user ? { id: user.id, email: user.email } : null,
      profile: profile ? {
        software_background: profile.software_background,
        hardware_experience: profile.hardware_experience,
        language_preference: profile.language_preference
      } : null,
      localStorage: {
        hasToken: !!token,
        hasUser: !!storedUser,
        hasProfile: !!storedProfile
      }
    });
  }, [isAuthenticated, isLoading, user, profile]);

  return <OriginalNavbar {...props} />;
}
