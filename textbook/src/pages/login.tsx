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
  const API_BASE_URL = 'https://physical-ai-auth-backend.onrender.com';
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
      
      const response = await fetch(`${API_BASE_URL}/api/auth/sign-in/email`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
        }),
      });

      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        throw new Error('Server error - invalid response from backend.');
      }

      const data = await response.json();

      if (!response.ok) {
        // ✅ CRITICAL: Handle redirect if user is not found or credentials are invalid
        const errorMessage = data.message || '';
        if (response.status === 404 || 
            errorMessage.toLowerCase().includes('not found') || 
            errorMessage.toLowerCase().includes('invalid email')) {
          
          setError('❌ Account not found. Redirecting to signup...');
          setTimeout(() => {
            window.location.href = `${baseUrl}signup`;
          }, 2000);
          return;
        }

        if (response.status === 401) {
          throw new Error('Invalid email or password. Please check your credentials.');
        }
        throw new Error(errorMessage || 'Login failed');
      }

      // ✅ Store auth data
      if (data.token || data.session?.token) {
        localStorage.setItem('auth_token', data.token || data.session.token);
      }
      if (data.user) {
        localStorage.setItem('user', JSON.stringify(data.user));
        if (data.user.profile || data.profile) {
          localStorage.setItem('profile', JSON.stringify(data.user.profile || data.profile));
        }
      }

      window.localStorage.setItem('isLoggedIn', 'true');
      localStorage.setItem('just_logged_in', 'true');
      setSuccess('Login successful! Redirecting to home...');

      // ✅ Redirect to Home
      setTimeout(() => {
        window.location.href = baseUrl;
      }, 1000);

    } catch (err: any) {
      console.error('[Login] ❌ Error:', err);
      setError(err.message || 'Login failed. Please try again.');
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

          {error && <div className={styles.errorBox}>{error}</div>}
          {success && <div className={styles.successBox}>✅ {success}</div>}

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