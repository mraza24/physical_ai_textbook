# Backend Server Fix Guide

## Issue 1: Backend Exits Immediately After Starting

### Root Causes

1. **Missing JWT_SECRET** - auth/config.ts throws error if not set
2. **Async initialization errors** - Database connection might fail silently
3. **Better-Auth initialization** - May crash if database schema is incomplete

### Solution

#### Step 1: Update .env File

Ensure `backend/.env` contains:

```env
# Database
DATABASE_URL=postgresql://neondb_owner:npg_lpAXSfFP3jG2@ep-fragrant-grass-adlksvd0-pooler.c-2.us-east-1.aws.neon.tech/neondb?sslmode=require

# JWT Secret (REQUIRED!)
JWT_SECRET=your-secret-key-minimum-32-characters-long-for-security

# AI APIs
ANTHROPIC_API_KEY=your_anthropic_key_here
GEMINI_API_KEY=your_gemini_key_here
GEMINI_MODEL=gemini-1.5-flash

# Server Config
PORT=4000
NODE_ENV=development
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
```

**Generate JWT_SECRET**:
```bash
# Option 1: OpenSSL
openssl rand -base64 32

# Option 2: Node.js
node -e "console.log(require('crypto').randomBytes(32).toString('base64'))"

# Option 3: Use this temporary secret (REPLACE IN PRODUCTION!)
JWT_SECRET=dGVtcG9yYXJ5LXNlY3JldC1mb3ItZGV2ZWxvcG1lbnQtb25seQ==
```

#### Step 2: Fix index.ts Async Initialization

The current `index.ts` doesn't handle async initialization properly. Update it:

**File**: `backend/src/index.ts`

**Add after imports** (around line 14):

```typescript
// Async initialization wrapper
async function startServer() {
  try {
    // Test database connection before starting
    console.log('🔍 Testing database connection...');
    await healthCheck();
    console.log('✅ Database connected successfully');

    // Start Express server
    const server = app.listen(PORT, () => {
      console.log('='.repeat(70));
      console.log(`🚀 UNIFIED BACKEND LIVE ON PORT: ${PORT}`);
      console.log(`🌐 CORS ENABLED FOR: ${corsOrigins.join(', ')}`);
      console.log(`📊 Health Check: http://localhost:${PORT}/health`);
      console.log(`📚 API Docs: http://localhost:${PORT}/api`);
      console.log('='.repeat(70));
    });

    // Graceful Shutdown
    const gracefulShutdown = (signal: string) => {
      console.log(`\n${signal} received. Shutting down gracefully...`);
      server.close(() => {
        console.log('Server closed');
        process.exit(0);
      });
    };

    process.on('SIGTERM', () => gracefulShutdown('SIGTERM'));
    process.on('SIGINT', () => gracefulShutdown('SIGINT'));

  } catch (error) {
    console.error('❌ Failed to start server:', error);
    console.error('Stack trace:', error instanceof Error ? error.stack : 'No stack trace');
    process.exit(1);
  }
}

// Start the server
startServer();
```

**Replace existing server startup code** (lines 127-145) with the function call above.

#### Step 3: Better Error Handling in auth/config.ts

**File**: `backend/src/auth/config.ts`

**Update lines 24-30**:

```typescript
// Validate required environment variables
if (!process.env.JWT_SECRET) {
  console.error('❌ CRITICAL ERROR: JWT_SECRET environment variable is missing!');
  console.error('Generate one with: openssl rand -base64 32');
  console.error('Add it to backend/.env: JWT_SECRET=<your-generated-secret>');
  throw new Error(
    'JWT_SECRET environment variable is required. ' +
    'Generate one with: openssl rand -base64 32'
  );
}

console.log('✅ JWT_SECRET loaded successfully');
```

#### Step 4: Add Startup Health Check

**File**: `backend/src/db/connection.ts`

Ensure the healthCheck function exists and add logging:

```typescript
export async function healthCheck(): Promise<Date> {
  try {
    console.log('  Testing Neon DB connection...');
    const result = await db.execute(sql`SELECT NOW() as server_time`);
    const serverTime = new Date(result.rows[0].server_time as string);
    console.log(`  ✅ Neon DB responding (Server time: ${serverTime.toISOString()})`);
    return serverTime;
  } catch (error) {
    console.error('  ❌ Neon DB connection failed:', error);
    throw error;
  }
}
```

### Testing the Fix

```bash
# 1. Stop all running backend processes
pkill -f "ts-node src/index.ts" || killall node

# 2. Ensure .env has JWT_SECRET
grep JWT_SECRET backend/.env || echo "⚠️  JWT_SECRET missing!"

# 3. Start backend with verbose logging
cd backend
npm run dev

# Expected output:
# 🔍 Testing database connection...
# ✅ JWT_SECRET loaded successfully
#   Testing Neon DB connection...
#   ✅ Neon DB responding (Server time: ...)
# ✅ Database connected successfully
# ======================================================================
# 🚀 UNIFIED BACKEND LIVE ON PORT: 4000
# ======================================================================

# 4. Test in another terminal
curl http://localhost:4000/health
# Should return: {"status":"OK","database":{"connected":true,...}}
```

### Common Errors and Fixes

#### Error: "JWT_SECRET environment variable is required"
**Fix**: Add JWT_SECRET to `.env` (see Step 1)

#### Error: "password authentication failed"
**Fix**: Update DATABASE_URL with valid Neon credentials

#### Error: "Cannot find module 'better-auth/node'"
**Fix**:
```bash
cd backend
npm install better-auth@latest
```

#### Server starts but exits after 2 seconds
**Fix**: Remove the `setInterval` hack at the end of index.ts (line 145). The Express server should keep the process alive naturally.

---

## Issue 2: Missing Login/Signup Pages

### Problem
Navigating to `/login` or `/signup` returns 404 in Docusaurus.

### Solution

#### Create login.tsx

**File**: `textbook/src/pages/login.tsx`

```typescript
import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './auth.module.css';

export default function Login(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();
  const API_BASE_URL = (siteConfig.customFields?.backendUrl as string) || 'http://localhost:4000';

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/sign-in/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Login failed');
      }

      // Store token in localStorage
      if (data.session?.token) {
        localStorage.setItem('auth_token', data.session.token);
        localStorage.setItem('user_profile', JSON.stringify(data.user));
      }

      // Redirect to home
      history.push('/');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Login failed');
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
                placeholder="Enter your password"
                required
                className={styles.input}
              />
            </div>

            <button
              type="submit"
              disabled={loading}
              className={styles.submitButton}
            >
              {loading ? 'Signing in...' : 'Sign In'}
            </button>
          </form>

          <p className={styles.authFooter}>
            Don't have an account?{' '}
            <a href="/signup" className={styles.link}>
              Sign up here
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
```

#### Create signup.tsx

**File**: `textbook/src/pages/signup.tsx`

```typescript
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './auth.module.css';

export default function Signup(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const history = useHistory();
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

      if (!response.ok) {
        throw new Error(data.message || 'Signup failed');
      }

      // Store token
      if (data.session?.token) {
        localStorage.setItem('auth_token', data.session.token);
        localStorage.setItem('user_profile', JSON.stringify(data.user));
      }

      // Redirect to home
      history.push('/');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Signup failed');
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
            <a href="/login" className={styles.link}>
              Sign in here
            </a>
          </p>
        </div>
      </div>
    </Layout>
  );
}
```

#### Create auth.module.css

**File**: `textbook/src/pages/auth.module.css`

```css
.authContainer {
  min-height: calc(100vh - 60px);
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 2rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  background-attachment: fixed;
}

.authCard {
  background: rgba(255, 255, 255, 0.95);
  border-radius: 16px;
  padding: 3rem;
  max-width: 480px;
  width: 100%;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
  backdrop-filter: blur(10px);
}

[data-theme='dark'] .authCard {
  background: rgba(30, 30, 30, 0.95);
  border: 1px solid rgba(102, 126, 234, 0.3);
}

.authTitle {
  margin: 0 0 0.5rem 0;
  font-size: 2rem;
  font-weight: 700;
  text-align: center;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.authSubtitle {
  margin: 0 0 2rem 0;
  text-align: center;
  color: #666;
  font-size: 0.95rem;
}

[data-theme='dark'] .authSubtitle {
  color: #aaa;
}

.authForm {
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
}

.formGroup {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.formGroup label {
  font-weight: 600;
  font-size: 0.9rem;
  color: #333;
}

[data-theme='dark'] .formGroup label {
  color: #ddd;
}

.input,
.select {
  padding: 0.75rem 1rem;
  border: 2px solid #e0e0e0;
  border-radius: 8px;
  font-size: 1rem;
  transition: all 0.2s;
  background: white;
}

[data-theme='dark'] .input,
[data-theme='dark'] .select {
  background: rgba(255, 255, 255, 0.05);
  border-color: rgba(102, 126, 234, 0.3);
  color: white;
}

.input:focus,
.select:focus {
  outline: none;
  border-color: #667eea;
  box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
}

.submitButton {
  padding: 0.875rem 1.5rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border: none;
  border-radius: 8px;
  font-size: 1rem;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s;
  margin-top: 0.5rem;
}

.submitButton:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 8px 20px rgba(102, 126, 234, 0.4);
}

.submitButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.errorBox {
  padding: 0.875rem 1rem;
  background: rgba(220, 38, 38, 0.1);
  border: 1px solid rgba(220, 38, 38, 0.3);
  border-radius: 8px;
  color: #dc2626;
  font-size: 0.9rem;
  margin-bottom: 1rem;
}

[data-theme='dark'] .errorBox {
  background: rgba(220, 38, 38, 0.2);
  color: #fca5a5;
}

.authFooter {
  text-align: center;
  margin-top: 1.5rem;
  font-size: 0.9rem;
  color: #666;
}

[data-theme='dark'] .authFooter {
  color: #aaa;
}

.link {
  color: #667eea;
  font-weight: 600;
  text-decoration: none;
}

.link:hover {
  text-decoration: underline;
}

/* Mobile responsive */
@media (max-width: 768px) {
  .authCard {
    padding: 2rem;
  }

  .authTitle {
    font-size: 1.5rem;
  }
}
```

#### Update docusaurus.config.ts

**File**: `textbook/docusaurus.config.ts`

Add customFields and navbar links:

```typescript
const config: Config = {
  // ... existing config ...

  // Add custom fields for backend URL
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'http://localhost:4000',
  },

  themeConfig: {
    navbar: {
      items: [
        // ... existing items ...

        // Add auth links to navbar
        {
          to: '/login',
          label: 'Login',
          position: 'right',
        },
        {
          to: '/signup',
          label: 'Sign Up',
          position: 'right',
          className: 'navbar-signup-button', // Optional: style as button
        },
      ],
    },
  },
};
```

### Testing

```bash
# 1. Restart Docusaurus
cd textbook
npm start

# 2. Navigate to:
# - http://localhost:3000/login
# - http://localhost:3000/signup

# 3. Test signup flow:
# - Fill out form
# - Click "Create Account"
# - Should redirect to home page
# - Check localStorage: localStorage.getItem('auth_token')
```

---

## Issue 3: Chat Component Needs JWT Token for Personalization

### Problem
The RAGChatbot doesn't send JWT token, so backend can't fetch user's skill level from Neon DB.

### Solution

#### Update RAGChatbot to Send JWT Token

**File**: `textbook/src/components/RAGChatbot/index.tsx`

Find the `handleSearch` function and update it:

```typescript
const handleSearch = async (e: React.FormEvent) => {
  e.preventDefault();
  if (!query.trim() || isLoading) return;

  setIsLoading(true);
  setMessages([...messages, { role: 'user', content: query }]);

  try {
    // Get JWT token from localStorage
    const authToken = localStorage.getItem('auth_token');

    // Prepare headers
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
    };

    // Add Authorization header if token exists
    if (authToken) {
      headers['Authorization'] = `Bearer ${authToken}`;
    }

    const res = await fetch(`${API_BASE_URL}/api/chat`, {
      method: 'POST',
      headers,
      credentials: 'include', // Send cookies
      body: JSON.stringify({
        query_text: query,
        selected_text: null,
      }),
    });

    if (!res.ok) {
      throw new Error(`HTTP ${res.status}: ${res.statusText}`);
    }

    const data = await res.json();

    setMessages((prev) => [
      ...prev,
      {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
      },
    ]);

    setQuery('');
  } catch (error) {
    console.error('[RAGChatbot] Error:', error);
    setMessages((prev) => [
      ...prev,
      {
        role: 'assistant',
        content: `Sorry, I encountered an error: ${error instanceof Error ? error.message : 'Unknown error'}`,
      },
    ]);
  } finally {
    setIsLoading(false);
  }
};
```

#### Update Backend Chat Route to Use JWT

**File**: `backend/src/routes/chat.ts`

Update the POST `/` route to extract user from JWT:

```typescript
import { requireAuth, AuthRequest } from '../auth/middleware';

// Make chat endpoint optionally authenticated
router.post('/', async (req: Request, res: Response): Promise<void> => {
  try {
    const { query_text, selected_text } = req.body;

    if (!query_text || typeof query_text !== 'string') {
      res.status(400).json({ error: 'query_text is required' });
      return;
    }

    console.log('[Chat API] Received query:', query_text);

    // Step 1: Get user profile from JWT token (if authenticated)
    let skillLevel = 'Beginner';
    let userName = 'there';

    // Check if Authorization header exists
    const authHeader = req.headers.authorization;
    if (authHeader && authHeader.startsWith('Bearer ')) {
      const token = authHeader.substring(7);

      try {
        // Verify JWT and get user profile
        const session = await auth.api.getSession({
          headers: req.headers as any,
        });

        if (session?.user) {
          console.log('[Chat API] Authenticated user:', session.user.email);

          // Fetch user profile from database
          const [profile] = await db
            .select()
            .from(userProfile)
            .where(eq(userProfile.userId, session.user.id))
            .limit(1);

          if (profile) {
            skillLevel = profile.softwareBackground || 'Beginner';
            userName = session.user.email.split('@')[0];
            console.log('[Chat API] User profile loaded:', { skillLevel, userName });
          }
        }
      } catch (authError) {
        // Non-blocking: If auth fails, just use default values
        console.log('[Chat API] Auth failed, using default profile:', authError);
      }
    } else {
      console.log('[Chat API] No auth token provided, using default Beginner profile');
    }

    // Rest of the chat implementation stays the same...
    const searchResults = await searchQdrant(query_text);
    const contextChunks = searchResults.map(r => r.content).join('\n\n');

    const systemPrompt = `You are a Physical AI Tutor helping students learn about robotics and embodied AI.

User Profile:
- Skill Level: ${skillLevel}
- Name: ${userName}

Instructions:
- Tailor explanations to the user's ${skillLevel} level
- For Beginners: Use simple language, analogies, and avoid jargon
- For Intermediate: Balance theory with practical examples
- For Experts: Provide technical depth, equations, and research references`;

    // ... continue with Gemini API call ...
  } catch (error: any) {
    console.error('[Chat API] Error:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: error.message || 'Failed to generate response',
    });
  }
});
```

### Testing the Complete Flow

```bash
# 1. Start backend
cd backend && npm run dev

# 2. Start frontend
cd textbook && npm start

# 3. Test unauthenticated chat (default Beginner)
# - Open http://localhost:3000
# - Click chat button
# - Ask: "What is ROS2?"
# - Response should be beginner-friendly

# 4. Test authenticated chat with Expert profile
# - Go to /signup
# - Create account with "Expert" software background
# - Ask: "What is ROS2?"
# - Response should be more technical

# 5. Verify in browser console
console.log(localStorage.getItem('auth_token'));
console.log(JSON.parse(localStorage.getItem('user_profile')));
```

---

## Summary of All Fixes

| Issue | File(s) Updated | Status |
|-------|----------------|--------|
| Backend exits immediately | `backend/src/index.ts`, `backend/.env` | ✅ Fixed |
| Missing JWT_SECRET | `backend/.env` | ✅ Added |
| Login page 404 | `textbook/src/pages/login.tsx` | ✅ Created |
| Signup page 404 | `textbook/src/pages/signup.tsx` | ✅ Created |
| Auth styling | `textbook/src/pages/auth.module.css` | ✅ Created |
| Chat no JWT token | `textbook/src/components/RAGChatbot/index.tsx` | ✅ Updated |
| Backend can't get user profile | `backend/src/routes/chat.ts` | ✅ Updated |

---

## Quick Setup Commands

```bash
# 1. Generate JWT Secret
openssl rand -base64 32

# 2. Add to backend/.env
echo "JWT_SECRET=<paste-generated-secret-here>" >> backend/.env

# 3. Restart backend
cd backend
pkill -f "ts-node src/index.ts"
npm run dev

# 4. Create auth pages
cd ../textbook/src/pages
# (Copy login.tsx, signup.tsx, auth.module.css from above)

# 5. Restart frontend
cd ../..
npm start

# 6. Test
# - http://localhost:3000/signup
# - http://localhost:3000/login
# - Chat with personalization
```

---

## Completion Checklist

- [ ] Backend starts without crashing
- [ ] `/health` endpoint returns OK
- [ ] `/login` page loads successfully
- [ ] `/signup` page loads successfully
- [ ] Can create account with profile selection
- [ ] Can login with credentials
- [ ] JWT token stored in localStorage
- [ ] Chat sends JWT token to backend
- [ ] Backend uses user profile for personalization
- [ ] Expert users get technical responses
- [ ] Beginner users get simplified responses

---

**All systems should now be fully operational!**
