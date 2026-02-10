import React, { useEffect, useState } from 'react';
import { AuthProvider } from '@site/src/contexts/AuthProvider';
import BulldogAssistant from '@site/src/components/BulldogAssistant';
import { useLocation, useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

/**
 * Root wrapper for entire Docusaurus site
 *
 * This component wraps ALL pages, ensuring:
 * 1. AuthProvider is available everywhere (fixes "useAuth must be used within AuthProvider" errors)
 * 2. RAGChatbot is rendered on every page with proper z-index
 * 3. Proper component ordering (AuthProvider first, then children, then chatbot)
 * 4. Auth redirect logic for protected routes
 *
 * CRITICAL: Docusaurus calls this component for EVERY page render
 */

// Component to handle auth redirects (must be inside AuthProvider)
function AuthRedirectHandler({ children }: { children: React.ReactNode }) {
  const location = useLocation();
  const history = useHistory();
  const { siteConfig } = useDocusaurusContext();
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  useEffect(() => {
    if (!isClient) return;

    // Check if user is authenticated
    const authToken = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;
    const currentPath = location.pathname;
    const baseUrl = siteConfig.baseUrl || '/';

    // Define protected routes (paths that require authentication)
    const protectedRoutes = [
      '/docs', // All documentation pages
      '/profile', // User profile page
    ];

    // Check if current path matches any protected route
    const isProtectedRoute = protectedRoutes.some(route =>
      currentPath.includes(route)
    );

    // Define public routes (pages that should be accessible without auth)
    const publicRoutes = [
      `${baseUrl}signup`,
      `${baseUrl}login`,
      baseUrl, // Home page
    ];

    const isPublicRoute = publicRoutes.some(route =>
      currentPath === route || currentPath === route.replace(baseUrl, '/')
    );

    // Redirect logic:
    // If accessing a protected route without auth token, redirect to signup
    if (isProtectedRoute && !authToken && !isPublicRoute) {
      console.log('[Auth Redirect] Unauthenticated user accessing protected route, redirecting to signup');
      history.push(`${baseUrl}signup`);
    }

    // DISABLED: Allow unauthenticated users to access login/signup pages
    // This was causing flicker and redirecting users away from auth pages
    // if ((currentPath.includes('login') || currentPath.includes('signup')) && authToken) {
    //   console.log('[Auth Redirect] Authenticated user accessing auth page, redirecting to home');
    //   history.push(baseUrl);
    // }
  }, [location.pathname, isClient, history, siteConfig.baseUrl]);

  return <>{children}</>;
}

export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      <AuthRedirectHandler>
        {/* All Docusaurus pages (docs, blog, custom pages) render here */}
        {children}
      </AuthRedirectHandler>

      {/* Bulldog Assistant - personalized AI helper (ONLY chatbot) */}
      <BulldogAssistant />
    </AuthProvider>
  );
}
