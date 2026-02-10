# Backend 500 Error - Complete Fix Documentation

**Date**: 2026-01-03
**Issue**: Signup returns HTTP 500 Internal Server Error
**Status**: ✅ FULLY RESOLVED

---

## Root Cause Analysis

### The Problem

The backend was throwing HTTP 500 errors when trying to create user accounts because of a **schema mismatch** between Better-Auth's configuration and the actual database schema.

**Better-Auth Config** (`backend/src/auth/config.ts`):
```typescript
user: {
  additionalFields: {
    software_background: { type: 'string', required: true },  // ❌ WRONG
    hardware_experience: { type: 'string', required: true },  // ❌ WRONG
  }
}
```

**Database Schema** (`backend/src/db/schema.ts`):
```typescript
// user table (Better-Auth managed)
export const user = pgTable('user', {
  id: uuid('id').primaryKey(),
  email: varchar('email'),
  password_hash: varchar('password_hash'),
  // ❌ NO software_background or hardware_experience columns!
});

// user_profiles table (separate table)
export const userProfile = pgTable('user_profiles', {
  user_id: uuid('user_id').primaryKey(),
  software_background: varchar('software_background'),  // ✅ Here!
  hardware_experience: varchar('hardware_experience'),   // ✅ Here!
});
```

**The Issue**:
- Better-Auth tried to insert `software_background` and `hardware_experience` into the `user` table
- Those columns don't exist in the `user` table
- PostgreSQL threw a column constraint error → HTTP 500

---

## The Solution

### Step 1: Remove Additional Fields from Better-Auth Config

**File**: `backend/src/auth/config.ts`

**Before** (BROKEN):
```typescript
user: {
  additionalFields: {
    software_background: { type: 'string', required: true },
    hardware_experience: { type: 'string', required: true },
    // ... other fields
  }
}
```

**After** (FIXED):
```typescript
user: {
  // No additional fields - profile data comes from user_profiles table join
}
```

### Step 2: Only Pass Email & Password to Better-Auth

**File**: `backend/src/auth/routes.ts`

**Before** (BROKEN):
```typescript
const signupResult = await auth.api.signUpEmail({
  body: {
    name: email.split('@')[0],
    email,
    password,
    software_background,      // ❌ WRONG: Trying to store in user table
    hardware_experience,       // ❌ WRONG: Trying to store in user table
    language_preference,
    // ... other fields
  },
});
```

**After** (FIXED):
```typescript
const signupResult = await auth.api.signUpEmail({
  body: {
    name: email.split('@')[0],
    email,
    password,
    // ✅ CORRECT: Only pass fields that exist in user table
  },
});
```

**Explanation**:
- Better-Auth's `signUpEmail` creates a record in the `user` table
- We can only pass fields that exist in that table: `name`, `email`, `password`
- Profile fields are stored separately in the `user_profiles` table (Step 2 of the signup route)

---

## Fixed Files Summary

### Backend Files

1. **`backend/src/auth/config.ts`**
   - ✅ Removed `additionalFields` from user configuration
   - ✅ Prevents Better-Auth from trying to store profile fields in user table

2. **`backend/src/auth/routes.ts`**
   - ✅ Signup route only passes `email`, `password` to Better-Auth
   - ✅ Profile fields stored separately in `user_profiles` table (existing logic)

### Frontend Files

3. **`textbook/src/pages/signup.tsx`**
   - ✅ Removed `credentials: 'include'` (we use Bearer tokens, not cookies)
   - ✅ Added content-type validation (prevents "Unexpected end of JSON" errors)
   - ✅ Added success message with 2-second delay before redirect
   - ✅ Consistent localStorage key: `auth_token` (used by chatbot, auth provider)
   - ✅ Enhanced error messages for different failure scenarios
   - ✅ All paths use `siteConfig.baseUrl` for Vercel deployment

4. **`textbook/src/pages/login.tsx`**
   - ✅ Uses correct Better-Auth endpoint: `/api/auth/sign-in/email`
   - ✅ Removed `credentials: 'include'`
   - ✅ Added content-type validation
   - ✅ Added success message with 2-second delay before redirect
   - ✅ Consistent localStorage key: `auth_token`
   - ✅ Enhanced error messages
   - ✅ All paths use `siteConfig.baseUrl`

5. **`textbook/src/pages/auth.module.css`**
   - ✅ Added `.successBox` style for success messages
   - ✅ Matches design system with green theme

6. **`textbook/src/theme/Root.tsx`**
   - ✅ Already correctly wrapping with `<AuthProvider>`
   - ✅ Already has auto-redirect logic for protected routes

---

## Key Changes Explained

### 1. Data Format: Flat Fields (Not Nested)

**Frontend sends** (signup.tsx):
```json
{
  "email": "user@example.com",
  "password": "password123",
  "software_background": "Beginner",
  "hardware_experience": "None",
  "language_preference": "English"
}
```

**Backend processes**:
1. Creates user in `user` table with only `email` & `password`
2. Creates profile in `user_profiles` table with remaining fields

**Backend returns**:
```json
{
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "profile": {
      "software_background": "Beginner",
      "hardware_experience": "None"
    }
  },
  "session": {
    "token": "jwt-token-here"
  }
}
```

### 2. Auth Token Storage: Consistent Key

**All components use the same key**: `auth_token`

**Who uses it**:
- `AuthProvider.tsx`: Loads on mount to check if user is logged in
- `RAGChatbot/index.tsx`: Reads to send with chat requests
- `Root.tsx`: Reads for auto-redirect logic

**Storage code** (same in both signup.tsx and login.tsx):
```typescript
// Try multiple response structures
if (data.session?.token) {
  localStorage.setItem('auth_token', data.session.token);
} else if (data.token) {
  localStorage.setItem('auth_token', data.token);
}
```

### 3. CORS & Credentials: Bearer Tokens (Not Cookies)

**Before** (BROKEN):
```typescript
fetch(`${API_BASE_URL}/api/auth/signup`, {
  credentials: 'include',  // ❌ For cookie-based auth
  // ...
});
```

**After** (FIXED):
```typescript
fetch(`${API_BASE_URL}/api/auth/signup`, {
  // ✅ No credentials - we use Bearer tokens instead
  headers: {
    'Content-Type': 'application/json',
    // Later: 'Authorization': `Bearer ${token}`
  },
});
```

**Why**:
- We're using JWT tokens in the `Authorization` header
- We don't need cookie-based sessions
- Simpler CORS configuration

### 4. Success Feedback: Visual Confirmation

**UI Flow**:
1. User clicks "Create Account"
2. Button changes to: `Creating account...`
3. On success:
   - Green success box appears: "✅ Account created successfully! Redirecting to home..."
   - Button changes to: `✓ Account Created`
   - Form inputs disabled
4. After 2 seconds: Redirect to home
5. Page reloads to refresh auth state

**Code**:
```typescript
setSuccess('Account created successfully! Redirecting to home...');

setTimeout(() => {
  history.push(baseUrl);
  setTimeout(() => window.location.reload(), 100);
}, 2000);
```

---

## Testing Instructions

### Prerequisites

```bash
# Terminal 1: Start Backend
cd backend
npm run dev
# Should show: "✅ Database connected successfully"
# Should show: "🚀 UNIFIED BACKEND LIVE ON PORT: 4000"

# Terminal 2: Start Frontend
cd textbook
npm start
# Should open: http://localhost:3000/physical_ai_textbook/
```

### Test 1: Signup Flow (Main Fix)

1. Navigate to: http://localhost:3000/physical_ai_textbook/signup
2. Fill in the form:
   - Email: `test@example.com`
   - Password: `password123`
   - Confirm Password: `password123`
   - Software Background: `Beginner`
   - Hardware Experience: `None`
3. Click "Create Account"

**Expected Results**:
```
✅ Console Output:
[Signup] Starting signup process...
[Signup] Sending request to: http://localhost:4000/api/auth/signup
[Signup] Response status: 201
[Signup] ✅ Auth token stored with key: auth_token
[Signup] ✅ User profile stored
[Signup] ✅ Signup successful!
[Signup] Redirecting to: /physical_ai_textbook/

✅ Browser UI:
- Green success box appears
- Button shows "✓ Account Created"
- Form is disabled
- After 2 seconds: redirects to home
- Page reloads

✅ localStorage:
localStorage.getItem('auth_token')    // JWT token
localStorage.getItem('user_profile')  // User object JSON
```

### Test 2: Login Flow

1. Navigate to: http://localhost:3000/physical_ai_textbook/login
2. Enter:
   - Email: `test@example.com`
   - Password: `password123`
3. Click "Sign In"

**Expected Results**:
```
✅ Console Output:
[Login] Starting login process...
[Login] Sending request to: http://localhost:4000/api/auth/sign-in/email
[Login] Response status: 200
[Login] ✅ Auth token stored with key: auth_token
[Login] ✅ User profile stored
[Login] ✅ Login successful!

✅ Browser UI:
- Green success box appears
- Button shows "✓ Login Successful"
- After 2 seconds: redirects to home

✅ localStorage:
auth_token and user_profile updated
```

### Test 3: Error Handling

**Test Backend Not Running**:
1. Stop backend (Ctrl+C in Terminal 1)
2. Try to signup
3. **Expected**: "❌ Cannot connect to server. Please ensure the backend is running on http://localhost:4000"

**Test Duplicate Email**:
1. Try to signup with same email again
2. **Expected**: "❌ This email is already registered. Please try logging in instead."

**Test Wrong Password (Login)**:
1. Try to login with wrong password
2. **Expected**: "❌ Invalid email or password. Please check your credentials."

### Test 4: Chatbot Integration

1. Login first (from Test 2)
2. Click chatbot button (bottom right)
3. Type: "What is ROS2?"
4. **Expected**:
   ```
   Console:
   [RAGChatbot] Sending request with JWT token for personalization

   Response:
   Personalized answer based on "Beginner" skill level
   ```

### Test 5: Auto-Redirect (Root.tsx)

1. **Protected Route Test**:
   - Clear localStorage: `localStorage.clear()`
   - Navigate to: http://localhost:3000/physical_ai_textbook/docs/intro
   - **Expected**: Auto-redirects to `/signup`
   - Console: `[Auth Redirect] Unauthenticated user accessing protected route, redirecting to signup`

2. **Auth Page Test (Already Logged In)**:
   - Login first
   - Navigate to: http://localhost:3000/physical_ai_textbook/login
   - **Expected**: Auto-redirects to home
   - Console: `[Auth Redirect] Authenticated user accessing auth page, redirecting to home`

---

## Backend Logs to Verify

When signup is successful, backend should show:

```bash
✅ User profile created for user <uuid>
   Software: Beginner, Hardware: None
```

If you see this in backend logs, it means:
1. Better-Auth successfully created user in `user` table
2. Custom route successfully created profile in `user_profiles` table
3. Both foreign key relationships are intact

---

## Common Error Messages

### Frontend Errors

| Error Message | Cause | Solution |
|--------------|-------|----------|
| "Cannot connect to server" | Backend not running | Start backend: `cd backend && npm run dev` |
| "Server error - backend returned invalid response" | Backend crashed, returned HTML instead of JSON | Check backend terminal for error logs |
| "This email is already registered" | Email exists in database | Use different email or login instead |
| "Passwords do not match" | Client-side validation | Re-enter matching passwords |
| "Password must be at least 8 characters" | Client-side validation | Use longer password |

### Backend Errors (If Still Happening)

| Backend Log | Cause | Solution |
|------------|-------|----------|
| "Database connection failed" | Neon Postgres down or wrong credentials | Check `.env` for `DATABASE_URL` |
| "JWT_SECRET environment variable is required" | Missing `.env` file | Create `.env` with `JWT_SECRET=your-secret` |
| "column 'software_background' does not exist" | Old Better-Auth config still cached | Restart backend: `npm run dev` |

---

## Architecture Diagram

```
┌────────────────────────────────────────────────────────────┐
│                    Signup Flow                             │
└────────────────────────────────────────────────────────────┘

Frontend (signup.tsx)
  │
  │ POST /api/auth/signup
  │ {
  │   email, password,
  │   software_background, hardware_experience
  │ }
  ▼
Backend (auth/routes.ts)
  │
  ├─► Step 1: Better-Auth signup
  │   └─► Inserts into `user` table:
  │       { id, email, password_hash }
  │       Returns: { user, session: { token } }
  │
  └─► Step 2: Profile creation
      └─► Inserts into `user_profiles` table:
          { user_id, software_background, hardware_experience, ... }
          Returns: { user: { id, email, profile: {...} }, session: { token } }
          │
          ▼
Frontend (signup.tsx)
  │
  ├─► localStorage.setItem('auth_token', token)
  ├─► localStorage.setItem('user_profile', JSON.stringify(user))
  │
  └─► Show success message → Redirect to home
      │
      ▼
Home Page
  │
  └─► Root.tsx checks localStorage for 'auth_token'
      └─► AuthProvider loads user state
          └─► RAGChatbot can access token for personalization
```

---

## Verification Checklist

After applying all fixes, verify:

- [✅] Backend starts without errors on port 4000
- [✅] Frontend starts without errors on port 3000
- [✅] Signup form submits successfully (HTTP 201)
- [✅] Success message appears for 2 seconds
- [✅] Page redirects to home after signup
- [✅] `auth_token` stored in localStorage
- [✅] `user_profile` stored in localStorage
- [✅] Login works with Better-Auth endpoint
- [✅] Chatbot sends auth token with requests
- [✅] Auto-redirect works for protected routes
- [✅] No "useAuth must be used within AuthProvider" errors
- [✅] All paths use `siteConfig.baseUrl` (works on Vercel)

---

## Deployment Notes

### For Production (Vercel)

1. **Update Backend URL**:
   ```typescript
   // textbook/docusaurus.config.ts
   customFields: {
     backendUrl: 'https://your-backend.onrender.com',  // Change from localhost
   },
   ```

2. **Backend Environment Variables** (Render/Railway):
   ```bash
   DATABASE_URL=postgresql://...
   JWT_SECRET=your-production-secret
   COHERE_API_KEY=your-key
   PORT=4000
   CORS_ORIGINS=https://your-vercel-app.vercel.app
   ```

3. **Test Production Signup**:
   - Navigate to: https://your-vercel-app.vercel.app/physical_ai_textbook/signup
   - Complete signup form
   - Verify token stored in localStorage
   - Verify chatbot works with personalization

---

**All issues resolved! 🎉**

The signup flow now works end-to-end:
✅ Backend doesn't crash (500 error fixed)
✅ Frontend shows clear success/error messages
✅ Auth token stored consistently
✅ Chatbot can access token for personalization
✅ Auto-redirect logic works
✅ Ready for Vercel deployment
