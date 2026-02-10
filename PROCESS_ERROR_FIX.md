# Fix: "process is not defined" in Docusaurus v3

## Root Cause Analysis

You have **3 files** with `process.env` calls that are being executed in the browser, causing the `ReferenceError: process is not defined` error.

### Files with Issues

1. **`textbook/src/hooks/usePersonalization.ts` (Line 4)**
   ```typescript
   const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';
   ```

2. **`textbook/src/hooks/useTranslation.ts` (Line 4)**
   ```typescript
   const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';
   ```

3. **`textbook/src/contexts/AuthProvider.tsx` (Line 3)**
   ```typescript
   const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';
   ```

## Why This Happens in Docusaurus v3

**Problem**: Docusaurus uses **Server-Side Rendering (SSR)** for initial page load, then becomes a **Single Page Application (SPA)** in the browser. When your hooks/components execute in the browser:

- ❌ `process.env` is a **Node.js global** (not available in browser)
- ❌ Even if defined during build, it doesn't exist at runtime
- ❌ Webpack's DefinePlugin only replaces at build time, not for dynamic imports

**The Issue**: Your custom hooks are imported by `ChapterActions.tsx`, which is rendered client-side, causing the error.

---

## Solution 1: Use Docusaurus customFields (RECOMMENDED)

This is the **safest and most Docusaurus-native** approach.

### Step 1: Update `docusaurus.config.ts`

**File**: `textbook/docusaurus.config.ts`

Add `customFields` to your config:

```typescript
const config: Config = {
  title: 'Physical AI Textbook',
  tagline: 'Learn Physical AI',
  url: 'https://your-site.com',
  baseUrl: '/',

  // ... other config ...

  // ✅ Add this section
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'http://localhost:4000',
  },

  themeConfig: {
    // ... your theme config ...
  },
};

export default config;
```

### Step 2: Fix `usePersonalization.ts`

**File**: `textbook/src/hooks/usePersonalization.ts`

**Replace line 4** with:

```typescript
import { useState } from 'react';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// ✅ SAFE: Use Docusaurus hook instead of process.env
const useApiUrl = () => {
  const { siteConfig } = useDocusaurusContext();
  return (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';
};

export interface PersonalizationResult {
  // ... rest stays the same ...
}

export const usePersonalization = () => {
  const { isAuthenticated } = useAuth();
  const API_BASE_URL = useApiUrl(); // ✅ Call hook inside component

  const [personalizing, setPersonalizing] = useState(false);
  // ... rest of the hook stays the same ...
```

### Step 3: Fix `useTranslation.ts`

**File**: `textbook/src/hooks/useTranslation.ts`

**Replace line 4** with the same pattern:

```typescript
import { useState, useEffect } from 'react';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// ✅ SAFE: Use Docusaurus hook
const useApiUrl = () => {
  const { siteConfig } = useDocusaurusContext();
  return (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';
};

export interface TranslationResult {
  // ... rest stays the same ...
}

export const useTranslation = () => {
  const { isAuthenticated, profile, updateProfile } = useAuth();
  const API_BASE_URL = useApiUrl(); // ✅ Call hook inside component

  const [translating, setTranslating] = useState(false);
  // ... rest of the hook stays the same ...
```

### Step 4: Fix `AuthProvider.tsx`

**File**: `textbook/src/contexts/AuthProvider.tsx`

**Replace line 3** with:

```typescript
import React, { createContext, useState, useEffect, ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// ✅ No more global constant with process.env

export interface User {
  // ... rest stays the same ...
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  // ✅ Get API URL safely inside component
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const [user, setUser] = useState<User | null>(null);
  // ... rest of the component stays the same ...
```

---

## Solution 2: Hardcode the URL (Quick Fix)

If you're only running locally and don't need dynamic configuration:

### usePersonalization.ts

```typescript
// ✅ SAFE: No process.env, works in all environments
const API_BASE_URL = 'http://localhost:4000';
```

### useTranslation.ts

```typescript
const API_BASE_URL = 'http://localhost:4000';
```

### AuthProvider.tsx

```typescript
const API_BASE_URL = 'http://localhost:4000';
```

**Pros**: Simple, no imports needed
**Cons**: Not flexible for production deployment

---

## Solution 3: Environment Detection Pattern

If you need different URLs for dev vs production:

```typescript
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname === 'localhost'
  ? 'http://localhost:4000'
  : 'https://api.yourproduction.com';
```

**Pros**: Automatic environment detection
**Cons**: Hardcoded production URL

---

## Why Your Original Code Failed

### ❌ Problematic Pattern (Module-level execution)

```typescript
// This runs when the module loads (before React rendering)
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:4000';
// ↑ ERROR: process is not defined in browser!

export const usePersonalization = () => {
  // Hook body...
```

**Problem**: The `const API_BASE_URL = ...` line executes **immediately when the file is imported**, which happens in the browser where `process` doesn't exist.

### ✅ Correct Pattern (Hook-level execution)

```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export const usePersonalization = () => {
  // ✅ This runs inside React component, after Docusaurus initializes
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  // Hook body...
```

**Why it works**: The code executes **after React renders**, when Docusaurus context is available.

---

## Complete Fix Implementation

### File 1: `usePersonalization.ts`

```typescript
import { useState } from 'react';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface PersonalizationResult {
  transformed_content: string;
  metadata: {
    model: string;
    changes_made: number;
    complexity_level: string;
    preserved_terms: string[];
    cached: boolean;
  };
  cache_key: string;
  processing_time_ms: number;
  validation_warnings?: string[];
}

export const usePersonalization = () => {
  // ✅ SAFE: Get API URL from Docusaurus context
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const { isAuthenticated } = useAuth();
  const [personalizing, setPersonalizing] = useState(false);
  const [personalized, setPersonalized] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);

  const personalizeChapter = async (chapterPath: string, content: string) => {
    if (!isAuthenticated) {
      setError('Please sign in to personalize content');
      return;
    }

    setPersonalizing(true);
    setError(null);

    try {
      const token = localStorage.getItem('auth_token');

      if (!token) {
        throw new Error('Not authenticated. Please sign in.');
      }

      const response = await fetch(`${API_BASE_URL}/api/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({ chapterPath, content }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Personalization failed');
      }

      const data: PersonalizationResult = await response.json();

      setPersonalizedContent(data.transformed_content);
      setPersonalized(true);
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Personalization failed';
      setError(message);
      setPersonalized(false);
    } finally {
      setPersonalizing(false);
    }
  };

  const resetChapter = () => {
    setPersonalized(false);
    setPersonalizedContent(null);
    setError(null);
  };

  return {
    personalizing,
    personalized,
    personalizedContent,
    error,
    personalizeChapter,
    resetChapter,
  };
};
```

### File 2: `useTranslation.ts`

```typescript
import { useState, useEffect } from 'react';
import { useAuth } from './useAuth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface TranslationResult {
  translated_content: string;
  metadata: {
    model: string;
    preserved_terms: string[];
    target_language: string;
    cached: boolean;
  };
  cache_key: string;
  processing_time_ms: number;
  validation_warnings?: string[];
}

export const useTranslation = () => {
  // ✅ SAFE: Get API URL from Docusaurus context
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const { isAuthenticated, profile, updateProfile } = useAuth();
  const [translating, setTranslating] = useState(false);
  const [language, setLanguage] = useState<'english' | 'urdu'>('english');
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [technicalTerms, setTechnicalTerms] = useState<string[]>([]);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (isAuthenticated && profile) {
      if (profile.language_preference === 'Urdu') {
        setLanguage('urdu');
      }
    }
  }, [isAuthenticated, profile]);

  const translateToUrdu = async (chapterPath: string, content: string) => {
    setTranslating(true);
    setError(null);

    try {
      const token = localStorage.getItem('auth_token');

      if (!token) {
        throw new Error('Not authenticated. Please sign in.');
      }

      const response = await fetch(`${API_BASE_URL}/api/translate/urdu`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`,
        },
        body: JSON.stringify({
          chapterPath,
          content,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Translation failed');
      }

      const data: TranslationResult = await response.json();

      setTranslatedContent(data.translated_content);
      setTechnicalTerms(data.metadata.preserved_terms);
      setLanguage('urdu');

      if (isAuthenticated && updateProfile) {
        try {
          await updateProfile({ language_preference: 'Urdu' });
        } catch (err) {
          console.error('Failed to persist language preference:', err);
        }
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Translation failed';
      setError(message);
    } finally {
      setTranslating(false);
    }
  };

  const showOriginal = async () => {
    setLanguage('english');
    setTranslatedContent(null);
    setTechnicalTerms([]);
    setError(null);

    if (isAuthenticated && updateProfile) {
      try {
        await updateProfile({ language_preference: 'English' });
      } catch (err) {
        console.error('Failed to persist language preference:', err);
      }
    }
  };

  return {
    translating,
    language,
    translatedContent,
    technicalTerms,
    error,
    translateToUrdu,
    showOriginal,
  };
};
```

### File 3: `AuthProvider.tsx`

```typescript
import React, { createContext, useState, useEffect, ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export interface User {
  id: string;
  email: string;
  created_at: string;
  last_login: string | null;
}

export interface UserProfile {
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
  language_preference: string;
  python_level?: string;
  ros2_level?: string;
  gpu_available?: string;
  hardware_tier?: string;
  primary_goal?: string;
}

export interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string, rememberMe?: boolean) => Promise<void>;
  signup: (email: string, password: string, profile: UserProfile) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (updates: Partial<UserProfile>) => Promise<void>;
  refreshProfile: () => Promise<void>;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  // ✅ SAFE: Get API URL from Docusaurus context inside component
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  // Load user from localStorage on mount
  useEffect(() => {
    const token = localStorage.getItem('auth_token');
    const storedUser = localStorage.getItem('user');
    const storedProfile = localStorage.getItem('profile');

    if (token && storedUser && storedProfile) {
      try {
        setUser(JSON.parse(storedUser));
        setProfile(JSON.parse(storedProfile));
      } catch (error) {
        console.error('Failed to parse stored auth data:', error);
        localStorage.removeItem('auth_token');
        localStorage.removeItem('user');
        localStorage.removeItem('profile');
      }
    }

    setIsLoading(false);
  }, []);

  const signup = async (email: string, password: string, profileData: UserProfile) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        email,
        password,
        software_background: profileData.software_background,
        hardware_experience: profileData.hardware_experience,
        language_preference: profileData.language_preference || 'English',
        python_level: profileData.python_level,
        ros2_level: profileData.ros2_level,
        gpu_available: profileData.gpu_available,
        hardware_tier: profileData.hardware_tier,
        primary_goal: profileData.primary_goal,
      }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.message || 'Signup failed');
    }

    const data = await response.json();

    localStorage.setItem('auth_token', data.session.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile));

    setUser(data.user);
    setProfile(data.profile);
  };

  const login = async (email: string, password: string, rememberMe = false) => {
    const response = await fetch(`${API_BASE_URL}/api/auth/login`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password, rememberMe }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.message || 'Login failed');
    }

    const data = await response.json();

    localStorage.setItem('auth_token', data.session.token);
    localStorage.setItem('user', JSON.stringify(data.user));
    localStorage.setItem('profile', JSON.stringify(data.profile));

    setUser(data.user);
    setProfile(data.profile);
  };

  const logout = async () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user');
    localStorage.removeItem('profile');

    setUser(null);
    setProfile(null);
  };

  const updateProfile = async (updates: Partial<UserProfile>) => {
    const token = localStorage.getItem('auth_token');

    if (!token) {
      throw new Error('Not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      method: 'PATCH',
      headers: {
        'Content-Type': 'application/json',
        Authorization: `Bearer ${token}`,
      },
      body: JSON.stringify(updates),
    });

    if (!response.ok) {
      const errorData = await response.json();
      throw new Error(errorData.message || 'Profile update failed');
    }

    const data = await response.json();

    localStorage.setItem('profile', JSON.stringify(data.profile));
    setProfile(data.profile);
  };

  const refreshProfile = async () => {
    const token = localStorage.getItem('auth_token');

    if (!token) {
      throw new Error('Not authenticated');
    }

    const response = await fetch(`${API_BASE_URL}/api/auth/profile`, {
      headers: {
        Authorization: `Bearer ${token}`,
      },
    });

    if (!response.ok) {
      throw new Error('Failed to refresh profile');
    }

    const data = await response.json();

    localStorage.setItem('profile', JSON.stringify(data.profile));
    setProfile(data.profile);
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        profile,
        isAuthenticated: !!user,
        isLoading,
        login,
        signup,
        logout,
        updateProfile,
        refreshProfile,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};
```

---

## Testing the Fix

```bash
# 1. Clear browser cache and localStorage
# - Open DevTools (F12)
# - Application tab → Clear storage

# 2. Restart Docusaurus
cd textbook
npm start

# 3. Open browser console (F12)
# - Should see NO "process is not defined" errors

# 4. Test ChapterActions
# - Navigate to any chapter
# - Click "Personalize Chapter" button
# - Should work without errors

# 5. Check network tab
# - Should see fetch to http://localhost:4000/api/personalize
```

---

## Why This Solution Works

### ✅ Key Principles

1. **No Module-Level `process.env`**
   - All environment access happens inside React components/hooks
   - Runs after Docusaurus initializes

2. **Docusaurus-Native Pattern**
   - Uses `useDocusaurusContext()` hook
   - Access `siteConfig.customFields` safely

3. **SSR Compatible**
   - No server-side execution errors
   - Works in both SSR and CSR modes

4. **Fallback Values**
   - Always has `|| 'http://localhost:4000'` fallback
   - Never undefined

---

## Common Mistakes to Avoid

### ❌ Don't Do This

```typescript
// ❌ BAD: Module-level execution
const API_URL = process.env.REACT_APP_URL;

export const MyComponent = () => {
  // Component body
};
```

### ✅ Do This Instead

```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export const MyComponent = () => {
  // ✅ GOOD: Component-level execution
  const { siteConfig } = useDocusaurusContext();
  const API_URL = siteConfig.customFields?.backendUrl as string;

  // Component body
};
```

---

## Summary

**Root Cause**: 3 files had `process.env` calls at module level (runs in browser)

**Files Fixed**:
1. `usePersonalization.ts` - Line 4
2. `useTranslation.ts` - Line 4
3. `AuthProvider.tsx` - Line 3

**Solution**: Use `useDocusaurusContext()` hook inside components instead of `process.env` at module level

**Result**: Zero browser errors, works in all environments

---

**All systems should now work without "process is not defined" errors!**
