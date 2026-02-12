import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './auth.module.css';

function SignupForm(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();

  // Get backend URL and baseUrl from config
const API_BASE_URL = 'https://physical-ai-auth-backend.onrender.com';
  const baseUrl = siteConfig.baseUrl || '/';

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState<'Beginner' | 'Intermediate' | 'Expert'>('Beginner');
  const [hardwareExperience, setHardwareExperience] = useState<'None' | 'Basic' | 'Advanced'>('None');
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess('');

    // Client-side validation
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    if (!email || !email.includes('@')) {
      setError('Please enter a valid email address');
      return;
    }

    setLoading(true);

    try {
      console.log('[Signup] 🚀 Starting signup process...');
      console.log('[Signup] Backend URL:', API_BASE_URL);
      console.log('[Signup] Payload:', {
        email,
        software_background: softwareBackground,
        hardware_experience: hardwareExperience,
        language_preference: 'English',
      });

      const endpoint = `${API_BASE_URL}/api/auth/signup`;
      console.log('[Signup] Calling endpoint:', endpoint);

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          software_background: softwareBackground,
          hardware_experience: hardwareExperience,
          language_preference: 'English',
        }),
      });

      console.log('[Signup] Response status:', response.status);
      console.log('[Signup] Response headers:', Object.fromEntries(response.headers.entries()));

      // CRITICAL: Check if response is successful (200, 201, 204)
      if (response.ok) {
        console.log('[Signup] ✅ Server accepted the request (status 2xx)');

        // Try to parse JSON response, but don't fail if empty
        let data = null;
        const contentType = response.headers.get('content-type');

        if (contentType && contentType.includes('application/json')) {
          try {
            const responseText = await response.text();
            if (responseText && responseText.trim() !== '') {
              data = JSON.parse(responseText);
              console.log('[Signup] Response data:', data);
            } else {
              console.log('[Signup] Empty JSON response (this is OK for 201 Created)');
            }
          } catch (parseError) {
            console.warn('[Signup] Could not parse JSON, but request was successful:', parseError);
          }
        } else {
          const responseText = await response.text();
          console.log('[Signup] Non-JSON response:', responseText.substring(0, 200));
        }

        // Store auth token if provided
        if (data?.session?.token) {
          localStorage.setItem('auth_token', data.session.token);
          console.log('[Signup] ✅ Auth token stored');
        } else if (data?.token) {
          localStorage.setItem('auth_token', data.token);
          console.log('[Signup] ✅ Auth token stored');
        } else {
          console.log('[Signup] ℹ️ No token in response (will login separately)');
        }

        // CRITICAL FIX: Store user data with correct keys that match AuthProvider.tsx
        // AuthProvider checks for 'user' and 'profile', NOT 'user_profile'!
        if (data?.user) {
          localStorage.setItem('user', JSON.stringify(data.user));
          console.log('[Signup] ✅ User stored with key: user');

          // Store profile separately if it exists
          if (data.user.profile) {
            localStorage.setItem('profile', JSON.stringify(data.user.profile));
            console.log('[Signup] ✅ Profile stored with key: profile');
          } else if (data.profile) {
            localStorage.setItem('profile', JSON.stringify(data.profile));
            console.log('[Signup] ✅ Profile stored with key: profile (from top-level)');
          }
        }

        // CRITICAL: Set simple login flag for immediate UI unlock (bypass server delay)
        localStorage.setItem('isLoggedIn', 'true');
        console.log('[Signup] ✅ Login flag set for immediate UI unlock');

        // Set flag for Bulldog to show welcome message
        localStorage.setItem('just_signed_up', 'true');

        // SUCCESS - Show clear message
        console.log('[Signup] ✅ SIGNUP SUCCESSFUL! Auto-logging in...');
        setSuccess('✅ Account Created! Welcome! Redirecting to homepage...');

        // Wait 1.5 seconds to show success message, then redirect to HOMEPAGE (user is already logged in)
        setTimeout(() => {
          console.log('[Signup] Redirecting to homepage (user auto-logged in)...');
          window.location.href = window.location.origin + baseUrl;
        }, 1500);

        return; // Exit early on success
      }

      // If response is not OK, try to get error details
      let errorData = null;
      const contentType = response.headers.get('content-type');

      if (contentType && contentType.includes('application/json')) {
        try {
          errorData = await response.json();
        } catch (e) {
          console.error('[Signup] Could not parse error response:', e);
        }
      } else {
        const errorText = await response.text();
        console.error('[Signup] Error response (non-JSON):', errorText);
        errorData = { message: errorText };
      }

      // Handle specific error status codes
      if (response.status === 409) {
        throw new Error('This email is already registered. Please try logging in instead.');
      } else if (response.status === 400) {
        throw new Error(errorData?.message || 'Invalid input. Please check your information.');
      } else if (response.status === 500) {
        console.error('[Signup] Server error:', errorData);
        throw new Error(errorData?.message || 'Server error. Please check backend logs.');
      } else {
        throw new Error(errorData?.message || `Signup failed with status ${response.status}`);
      }

    } catch (err) {
      console.error('[Signup] ❌ Error:', err);

      // User-friendly error messages
      let errorMessage = 'Signup failed. Please try again.';

      if (err instanceof Error) {
        if (err.message.includes('Failed to fetch') || err.message.includes('NetworkError')) {
           errorMessage = '❌ Cannot connect to backend. Please check your internet or server status.';        } else if (err.message.includes('already registered') || err.message.includes('already exists')) {
          errorMessage = '❌ Email already registered. Try logging in instead.';
        } else {
          errorMessage = `❌ ${err.message}`;
        }
      }

      setError(errorMessage);
      console.error('[Signup] Full error details:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create your Physical AI Textbook account">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Create Your Account</h1>
          <p className={styles.authSubtitle}>Join thousands of learners mastering Physical AI</p>

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
                placeholder="At least 8 characters"
                required
                className={styles.input}
                disabled={loading || !!success}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="confirmPassword">Confirm Password</label>
              <input
                id="confirmPassword"
                type="password"
                value={confirmPassword}
                onChange={(e) => setConfirmPassword(e.target.value)}
                placeholder="Re-enter your password"
                required
                className={styles.input}
                disabled={loading || !!success}
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="softwareBackground">Software Background</label>
              <select
                id="softwareBackground"
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value as any)}
                className={styles.select}
                disabled={loading || !!success}
              >
                <option value="Beginner">Beginner - New to programming</option>
                <option value="Intermediate">Intermediate - Some coding experience</option>
                <option value="Expert">Expert - Professional developer</option>
              </select>
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="hardwareExperience">Hardware Experience</label>
              <select
                id="hardwareExperience"
                value={hardwareExperience}
                onChange={(e) => setHardwareExperience(e.target.value as any)}
                className={styles.select}
                disabled={loading || !!success}
              >
                <option value="None">None - No robotics experience</option>
                <option value="Basic">Basic - Tinkered with Arduino/Raspberry Pi</option>
                <option value="Advanced">Advanced - Built robots before</option>
              </select>
            </div>

            <button
              type="submit"
              disabled={loading || !!success}
              className={styles.submitButton}
            >
              {loading ? 'Creating account...' : success ? '✓ Account Created' : 'Create Account'}
            </button>
          </form>

          <p className={styles.authFooter}>
            Already have an account?{' '}
            <a href={`${baseUrl}login`} className={styles.link}>
              Sign in here
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}

export default function Signup(): React.JSX.Element {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <SignupForm />}
    </BrowserOnly>
  );
}
