import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './auth.module.css';

function LoginForm() {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();

  // Get backend URL and baseUrl from config
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';
  const baseUrl = siteConfig.baseUrl || '/';

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess('');

    if (!email || !password) {
      setError('Please enter both email and password');
      return;
    }

    setLoading(true);

    try {
      console.log('[Login] Starting login process...');
      console.log('[Login] Sending request to:', `${API_BASE_URL}/api/auth/sign-in/email`);

      // CRITICAL: Better-Auth uses /api/auth/sign-in/email endpoint
      // See: https://www.better-auth.com/docs/api-reference/auth#email-password
      const response = await fetch(`${API_BASE_URL}/api/auth/sign-in/email`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        // NOTE: credentials: 'include' removed - we use Bearer tokens, not cookies
        body: JSON.stringify({
          email,
          password,
        }),
      });

      console.log('[Login] Response status:', response.status);
      console.log('[Login] Response headers:', {
        contentType: response.headers.get('content-type'),
      });

      // CRITICAL: Handle non-JSON responses (fixes "Unexpected end of JSON input" error)
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        const responseText = await response.text();
        console.error('[Login] Non-JSON response:', responseText);
throw new Error('Server error - backend returned an invalid response. Please try again later.');      }

      const data = await response.json();
      console.log('[Login] Response data:', data);

      if (!response.ok) {
        // Handle specific error cases
        if (response.status === 401) {
          throw new Error('Invalid email or password. Please check your credentials.');
        } else if (response.status === 404) {
          throw new Error('Account not found. Please sign up first.');
        } else if (response.status === 500) {
          console.error('[Login] Server error details:', data);
          throw new Error(data.message || 'Server error. Please try again later.');
        }

        throw new Error(data.message || data.error || 'Login failed');
      }

      // CRITICAL: Store auth token IMMEDIATELY using consistent key 'auth_token'
      // This key is used by: AuthProvider.tsx, RAGChatbot/index.tsx, Root.tsx
      if (data.token) {
        localStorage.setItem('auth_token', data.token);
        console.log('[Login] ✅ Auth token stored with key: auth_token');
      } else if (data.session?.token) {
        localStorage.setItem('auth_token', data.session.token);
        console.log('[Login] ✅ Auth token stored with key: auth_token (from session)');
      } else {
        console.warn('[Login] ⚠️ No token in response:', data);
      }

      // CRITICAL FIX: Store user data with correct keys that match AuthProvider.tsx
      // AuthProvider checks for 'user' and 'profile', NOT 'user_profile'!
      if (data.user) {
        localStorage.setItem('user', JSON.stringify(data.user));
        console.log('[Login] ✅ User stored with key: user');

        // Store profile separately if it exists
        if (data.user.profile) {
          localStorage.setItem('profile', JSON.stringify(data.user.profile));
          console.log('[Login] ✅ Profile stored with key: profile');
        } else if (data.profile) {
          localStorage.setItem('profile', JSON.stringify(data.profile));
          console.log('[Login] ✅ Profile stored with key: profile (from top-level)');
        }
      }

      // CORRECT REDIRECT: Use baseUrl for proper navigation
      window.localStorage.setItem('isLoggedIn', 'true');
      console.log('[Login] CORRECT REDIRECT: isLoggedIn=true set globally');

      // SUCCESS - Show feedback message
      console.log('[Login] ✅ Login successful!');
      setSuccess('Login successful! Redirecting to home...');

      // Set flag for Bulldog to show login confirmation
      localStorage.setItem('just_logged_in', 'true');

      // CORRECT REDIRECT: Navigate to home page with proper baseUrl
      setTimeout(() => {
  console.log('[Login] CORRECT REDIRECT: Navigating to home');
  // اسے بدل دیں
  window.location.href = baseUrl; 
}, 1000);

   } catch (err) {
  console.error('[Login] ❌ Error:', err);

  if (err instanceof Error && err.message.includes('not found')) {
      setError('❌ Account not found. Redirecting to signup...');
      setTimeout(() => {
          window.location.href = `${baseUrl}signup`; // سائن اپ پر بھیج دیں
      }, 2000);
      return;
  }
  // ... باقی ایرر ہینڈلنگ ویسے ہی رہنے دیں

      // User-friendly error messages
      let errorMessage = 'Login failed. Please try again.';

      if (err instanceof Error) {
        if (err.message.includes('Failed to fetch') || err.message.includes('NetworkError')) {
           errorMessage = '❌ Cannot connect to backend. Please check your internet or server status.';        } else if (err.message.includes('already registered') || err.message.includes('already exists')) {
        } else if (err.message.includes('Unexpected end of JSON') || err.message.includes('JSON')) {
          errorMessage = '❌ Server error - received invalid response. Please check if the backend is running correctly.';
        } else if (err.message.includes('Invalid email or password')) {
          errorMessage = '❌ Invalid email or password. Please check your credentials.';
        } else if (err.message.includes('not found')) {
          errorMessage = '❌ Account not found. Please sign up first.';
        } else {
          errorMessage = `❌ ${err.message}`;
        }
      }

      setError(errorMessage);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Login" description="Login to Physical AI Textbook">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Welcome Back</h1>
          <p className={styles.authSubtitle}>Sign in to continue your learning journey</p>

          {error && (
            <div className={styles.errorBox}>
              {error}
            </div>
          )}

          {success && (
            <div className={styles.successBox}>
              ✅ {success}
            </div>
          )}

          <form onSubmit={handleSubmit} className={styles.authForm}>
            <div className={styles.formGroup}>
              <label htmlFor="email">Email Address</label>
              <input
                id="email"
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                placeholder="your.email@example.com"
                required
                autoFocus
                className={styles.input}
                disabled={loading || !!success}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password">Password</label>
              <input
                id="password"
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                placeholder="Enter your password"
                required
                className={styles.input}
                disabled={loading || !!success}
              />
            </div>

            <button
              type="submit"
              disabled={loading || !!success}
              className={styles.submitButton}
            >
              {loading ? 'Signing in...' : success ? '✓ Login Successful' : 'Sign In'}
            </button>
          </form>

          <p className={styles.authFooter}>
            Don't have an account?{' '}
            <a href={`${baseUrl}signup`} className={styles.link}>
              Sign up here
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}

export default function Login() {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <LoginForm />}
    </BrowserOnly>
  );
}
