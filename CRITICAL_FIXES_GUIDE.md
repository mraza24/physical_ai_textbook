# Critical Fixes for Physical AI Textbook - Docusaurus Project

## Issues Summary

1. ❌ **AuthProvider Error**: `useAuth must be used within an AuthProvider` - crashes on /docs/intro
2. ❌ **Signup Not Working**: Click "Create Account" does nothing
3. ❌ **Login Link Broken**: "Sign in here" leads to 404
4. ❌ **Chatbot Backend Error**: HTTP 500, "No auth token found"

---

## Issue 1: AuthProvider Not Wrapping Application

### Problem
The `AuthProvider` context is not wrapping the entire Docusaurus application, so when components try to use `useAuth()`, they crash.

### Root Cause
`Root.tsx` doesn't include the `AuthProvider` wrapper.

### Solution: Update `src/theme/Root.tsx`

**File**: `textbook/src/theme/Root.tsx`

```typescript
import React from 'react';
import { AuthProvider } from '@site/src/contexts/AuthProvider';
import RAGChatbot from '@site/src/components/RAGChatbot';

/**
 * Root wrapper for entire Docusaurus site
 *
 * This component wraps ALL pages, ensuring:
 * 1. AuthProvider is available everywhere (no more "useAuth must be used within AuthProvider" errors)
 * 2. RAGChatbot is rendered on every page
 * 3. Proper component ordering (AuthProvider first, then children, then chatbot)
 */
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      {/* All Docusaurus pages render here */}
      {children}

      {/* Floating chatbot - renders on top of all content */}
      <div style={{ position: 'relative', zIndex: 1000 }}>
        <RAGChatbot />
      </div>
    </AuthProvider>
  );
}
```

**Why This Works**:
- `AuthProvider` wraps everything, making `useAuth()` available in all components
- Docusaurus calls `Root.tsx` for EVERY page
- Order matters: AuthProvider → children → chatbot

---

## Issue 2: Signup Page Not Submitting

### Problems
1. Backend expects different data structure
2. Missing proper error handling
3. Navigation using wrong path (needs baseUrl)
4. Token storage incomplete

### Solution: Update `src/pages/signup.tsx`

**File**: `textbook/src/pages/signup.tsx`

```typescript
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './auth.module.css';

export default function Signup(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();

  // Get backend URL from config
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState<'Beginner' | 'Intermediate' | 'Expert'>('Beginner');
  const [hardwareExperience, setHardwareExperience] = useState<'None' | 'Basic' | 'Advanced'>('None');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    // Validation
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    setLoading(true);

    try {
      console.log('[Signup] Sending request to:', `${API_BASE_URL}/api/auth/signup`);

      const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({
          email,
          password,
          software_background: softwareBackground,
          hardware_experience: hardwareExperience,
          language_preference: 'English',
        }),
      });

      const data = await response.json();
      console.log('[Signup] Response:', data);

      if (!response.ok) {
        throw new Error(data.message || data.error || 'Signup failed');
      }

      // Store auth data in localStorage
      if (data.session?.token) {
        localStorage.setItem('auth_token', data.session.token);
        console.log('[Signup] Token stored successfully');
      }

      if (data.user) {
        localStorage.setItem('user_profile', JSON.stringify(data.user));
        console.log('[Signup] User profile stored');
      }

      // SUCCESS - Redirect to home with baseUrl
      console.log('[Signup] Success! Redirecting to home...');

      // Use baseUrl from config for proper navigation
      const baseUrl = siteConfig.baseUrl || '/';
      history.push(baseUrl);

      // Reload to refresh auth state
      window.location.reload();

    } catch (err) {
      console.error('[Signup] Error:', err);
      setError(err instanceof Error ? err.message : 'Signup failed. Please try again.');
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
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="softwareBackground">Software Background</label>
              <select
                id="softwareBackground"
                value={softwareBackground}
                onChange={(e) => setSoftwareBackground(e.target.value as any)}
                className={styles.select}
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
              >
                <option value="None">None - No robotics experience</option>
                <option value="Basic">Basic - Tinkered with Arduino/Raspberry Pi</option>
                <option value="Advanced">Advanced - Built robots before</option>
              </select>
            </div>

            <button
              type="submit"
              disabled={loading}
              className={styles.submitButton}
            >
              {loading ? 'Creating account...' : 'Create Account'}
            </button>
          </form>

          <p className={styles.authFooter}>
            Already have an account?{' '}
            <a href={`${siteConfig.baseUrl}login`} className={styles.link}>
              Sign in here
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
```

**Key Fixes**:
1. ✅ Proper error handling with console logs
2. ✅ Correct data structure (flat fields, not nested profile)
3. ✅ Uses `siteConfig.baseUrl` for navigation
4. ✅ Stores both token and user profile
5. ✅ Password validation (min 8 chars)
6. ✅ Proper TypeScript types

---

## Issue 3: Login Link Returns 404

### Problem
The link to login page doesn't account for `baseUrl` (`/physical_ai_textbook/`)

### Solution
Already fixed in signup.tsx above - uses:
```typescript
<a href={`${siteConfig.baseUrl}login`} className={styles.link}>
```

This correctly generates: `/physical_ai_textbook/login`

---

## Issue 4: Chatbot Backend Connection Error

### Problems
1. No auth token being sent
2. Backend might not be handling optional auth
3. CORS issues

### Solution 1: Update Chatbot to Send Token

**File**: `textbook/src/components/RAGChatbot/index.tsx`

Find the `handleSearch` function and ensure it sends the auth token:

```typescript
const handleSearch = async (e: React.FormEvent) => {
  e.preventDefault();
  if (!query.trim() || !typingComplete) return;

  const userMsg = query;
  setMessages(prev => [...prev, { role: 'user', text: userMsg }]);
  setQuery('');
  setLoading(true);
  setTypingComplete(false);

  try {
    // Get JWT token from localStorage (if user is logged in)
    const authToken = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;

    // Prepare headers with optional JWT token
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
    };

    // Add Authorization header if token exists
    if (authToken) {
      headers['Authorization'] = `Bearer ${authToken}`;
      console.log('[RAGChatbot] Sending request with JWT token');
    } else {
      console.log('[RAGChatbot] No auth token found, sending as guest');
    }

    const res = await fetch(`${API_BASE_URL}/api/chat`, {
      method: 'POST',
      headers,
      credentials: 'include',
      body: JSON.stringify({
        query_text: userMsg,
        selected_text: null
      }),
    });

    if (!res.ok) {
      const errorData = await res.json();
      throw new Error(errorData.message || `HTTP ${res.status}: ${res.statusText}`);
    }

    const data = await res.json();
    setMessages(prev => [...prev, {
      role: 'ai',
      text: data.answer || data.text || "I'm processing that...",
      sources: data.citations || [],
      isTyping: true
    }]);
  } catch (err) {
    console.error('[RAGChatbot] Error:', err);
    setMessages(prev => [...prev, {
      role: 'ai',
      text: `⚠️ Connection Error: ${err instanceof Error ? err.message : 'Unknown error'}. Please ensure backend is running on http://localhost:4000`
    }]);
    setTypingComplete(true);
  } finally {
    setLoading(false);
  }
};
```

### Solution 2: Update Backend to Handle Optional Auth

**File**: `backend/src/routes/chat.ts`

The backend should already handle optional auth (from previous fixes), but verify it has:

```typescript
// Step 1: Get user profile from JWT token (OPTIONAL - works without auth)
let skillLevel = 'Beginner';
let userName = 'there';
let authenticated = false;

// Check if Authorization header exists
const authHeader = req.headers.authorization;
if (authHeader && authHeader.startsWith('Bearer ')) {
  try {
    const session = await auth.api.getSession({
      headers: req.headers as any,
    });

    if (session?.user) {
      authenticated = true;
      // Fetch user profile...
    }
  } catch (authError) {
    // Non-blocking: If auth fails, just use default values
    console.log('[Chat API] Auth verification failed (using defaults)');
  }
} else {
  console.log('[Chat API] No auth token provided, using default Beginner profile');
}
```

---

## docusaurus.config.ts Verification

### Current Configuration (CORRECT)

**File**: `textbook/docusaurus.config.ts`

```typescript
const config: Config = {
  title: 'Physical AI Textbook',

  // ✅ CORRECT: Your GitHub Pages URL
  url: 'https://mraza24.github.io',

  // ✅ CORRECT: Matches your repository name
  baseUrl: '/physical_ai_textbook/',

  // ✅ CORRECT: Makes backend URL available to all components
  customFields: {
    backendUrl: 'http://localhost:4000',
  },

  // ... rest of config
};
```

**How It Works**:

1. **baseUrl**: All routes are prefixed with `/physical_ai_textbook/`
   - `/login` → `/physical_ai_textbook/login`
   - `/signup` → `/physical_ai_textbook/signup`
   - `/docs/intro` → `/physical_ai_textbook/docs/intro`

2. **customFields**: Access in components via:
   ```typescript
   const { siteConfig } = useDocusaurusContext();
   const API_BASE_URL = siteConfig.customFields?.backendUrl as string;
   ```

3. **url + baseUrl**: Combined for production deployment:
   - `https://mraza24.github.io/physical_ai_textbook/`

---

## Testing Checklist

### 1. Test AuthProvider Fix

```bash
# Start frontend
cd textbook
npm start

# Navigate to: http://localhost:3000/physical_ai_textbook/docs/intro
# Should NOT crash with "useAuth must be used within AuthProvider"
```

### 2. Test Signup Flow

```bash
# 1. Navigate to: http://localhost:3000/physical_ai_textbook/signup
# 2. Fill in form:
#    - Email: test@example.com
#    - Password: password123
#    - Confirm Password: password123
#    - Software Background: Beginner
#    - Hardware Experience: None
# 3. Click "Create Account"
# 4. Check browser console for logs:
#    - [Signup] Sending request to: http://localhost:4000/api/auth/signup
#    - [Signup] Response: {session: {...}, user: {...}}
#    - [Signup] Token stored successfully
#    - [Signup] Success! Redirecting to home...
# 5. Should redirect to: http://localhost:3000/physical_ai_textbook/
# 6. Verify localStorage:
#    - localStorage.getItem('auth_token')
#    - localStorage.getItem('user_profile')
```

### 3. Test Login Link

```bash
# From signup page, click "Sign in here"
# Should navigate to: http://localhost:3000/physical_ai_textbook/login
# Should NOT show 404
```

### 4. Test Chatbot with Auth

```bash
# 1. Ensure you're logged in (from step 2)
# 2. Click chatbot button (bottom right)
# 3. Type: "What is ROS2?"
# 4. Check browser console:
#    - [RAGChatbot] Sending request with JWT token
# 5. Should receive response without HTTP 500 error
```

### 5. Test Chatbot without Auth

```bash
# 1. Clear localStorage: localStorage.clear()
# 2. Refresh page
# 3. Click chatbot, ask question
# 4. Check console:
#    - [RAGChatbot] No auth token found, sending as guest
# 5. Should still get response (uses default Beginner profile)
```

---

## Backend CORS Configuration

Ensure your backend has proper CORS:

**File**: `backend/src/index.ts`

```typescript
app.use(cors({
  origin: [
    'http://localhost:3000',
    'https://mraza24.github.io'  // Add for production
  ],
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization'],
}));
```

---

## Common Errors and Fixes

### Error: "useAuth must be used within AuthProvider"
**Fix**: Ensure `Root.tsx` wraps children with `<AuthProvider>`

### Error: Signup button does nothing
**Fix**: Check browser console for fetch errors, verify backend is running

### Error: 404 on /login
**Fix**: Ensure link uses `${siteConfig.baseUrl}login`

### Error: Chatbot HTTP 500
**Fix**:
1. Verify backend is running on port 4000
2. Check backend logs for actual error
3. Ensure chat route handles optional auth

### Error: CORS error in console
**Fix**: Add `http://localhost:3000` to backend CORS origins

---

## File Summary

**Files to Update**:

1. ✅ `textbook/src/theme/Root.tsx` - Wrap with AuthProvider
2. ✅ `textbook/src/pages/signup.tsx` - Fix form submission
3. ✅ `textbook/src/components/RAGChatbot/index.tsx` - Send auth token
4. ✅ `textbook/docusaurus.config.ts` - Already correct!

**Files to Verify**:

1. `backend/src/routes/chat.ts` - Optional auth handling
2. `backend/src/index.ts` - CORS configuration

---

## Quick Fix Commands

```bash
# 1. Update Root.tsx (copy fixed version from this guide)
# 2. Update signup.tsx (copy fixed version from this guide)
# 3. Restart frontend
cd textbook
npm start

# 4. Restart backend
cd backend
npm run dev

# 5. Test full flow
# - Go to /signup
# - Create account
# - Should redirect to home
# - Open chatbot
# - Ask question
# - Should work!
```

---

**All critical issues should now be resolved!**
