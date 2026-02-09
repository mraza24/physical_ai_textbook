# Part 2: Isolation Testing Suite - Complete Guide

**Purpose**: Pinpoint the exact source of failures by testing each component in isolation

---

## Prerequisites

```bash
# Navigate to backend directory
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/backend

# Ensure backend is NOT running (to avoid port conflicts for some tests)
# If backend is running, stop it with Ctrl+C
```

---

## Test 1: Database Connection Test (Drizzle ORM)

**What it tests**: Verifies Drizzle can connect to Neon and query tables

**Run**:
```bash
npx tsx test-db.ts
```

**Expected Output** (if working):
```
✅ Database connection: SUCCESS
✅ Drizzle ORM: WORKING
✅ Tables found: 4
✅ Required tables: ALL PRESENT
  - user
  - session
  - user_profiles
  - transformation_cache
```

**Expected Output** (if failing):
```
❌ DATABASE TEST FAILED:
Database health check failed: Cannot connect to Neon database.
  Possible causes:
  1. DATABASE_URL is incorrect or malformed
  2. Neon database is paused (wake it up at https://console.neon.tech)
  ...
```

**What to look for**:
- ✅ **SUCCESS**: Database connection works, all tables exist
- ❌ **FAIL - "fetch failed"**: DATABASE_URL is wrong or Neon is paused
- ❌ **FAIL - "tables missing"**: Need to run database migrations

---

## Test 2: Cohere API Key Validation

**What it tests**: Verifies Cohere API key is valid and not rate-limited

**Run**:
```bash
npx tsx test-cohere.ts
```

**Expected Output** (if working):
```
✅ API Key: VALID
✅ Generate endpoint: WORKING
✅ Chat endpoint: WORKING
✅ API Key Status: ACTIVE (2026)
```

**Expected Output** (if failing):
```
❌ Error: 401 Unauthorized
   Your API key is invalid or expired
```

OR

```
❌ Error: 429 Rate Limit Exceeded
   You have hit the trial API limit
```

**What to look for**:
- ✅ **SUCCESS**: API key ending in `jvSQ` works
- ❌ **401 Unauthorized**: Key is invalid/expired
- ❌ **429 Rate Limit**: Trial quota exhausted
- ❌ **402 Payment Required**: Need to upgrade plan

---

## Test 3: JWT Token Inspection

**What it tests**: Verifies JWT tokens have correct expiry dates (not defaulting to 2025)

**Run**:
```bash
npx tsx test-jwt.ts
```

**Expected Output** (if working):
```
✅ JWT Secret: SET
✅ Token Generation: WORKING
✅ Token Format: VALID
✅ Token Dates: VALID (2026)

Issued At (iat): 2026-01-03T12:34:56.789Z
Expires At (exp): 2026-01-04T12:34:56.789Z
Time until expiry: 24.00 hours
```

**Expected Output** (if failing):
```
❌ WARNING: Token is ALREADY EXPIRED!
   Expires At (exp): 2025-12-31T...
   (Expired date in 2025!)
```

**What to look for**:
- ✅ **SUCCESS**: Token issued in 2026, expires in 2026 (24 hours later)
- ❌ **EXPIRED**: Token expires before issue date
- ❌ **2025 DATES**: Tokens using old dates (system clock issue)

---

## Test 4: Auth Route Direct API Test

**What it tests**: Hits `/api/auth/signup` endpoint directly to see if 500 error occurs

**Prerequisites**:
```bash
# Backend MUST be running for this test
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/backend
npm run dev
```

**Run** (in separate terminal):
```bash
# Make script executable
chmod +x test-auth.sh

# Run test
./test-auth.sh
```

**OR use manual curl command**:
```bash
curl -v -X POST http://localhost:4000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test-123@example.com",
    "password": "testpassword123",
    "software_background": "Beginner",
    "hardware_experience": "None",
    "language_preference": "English"
  }'
```

**Expected Output** (if working):
```
HTTP/1.1 201 Created
Content-Type: application/json

{
  "user": {
    "id": "...",
    "email": "test-123@example.com",
    "profile": {
      "software_background": "Beginner",
      "hardware_experience": "None"
    }
  },
  "session": {
    "token": "eyJhbGci..."
  }
}
```

**Expected Output** (if failing):
```
HTTP/1.1 500 Internal Server Error

{
  "error": "Internal Server Error",
  "message": "..."
}
```

**What to look for**:
- ✅ **HTTP 201**: Signup works end-to-end
- ❌ **HTTP 500**: This is the error! Check backend logs for details
- ❌ **HTTP 400**: Validation error (check request body format)

---

## Test 5: Docusaurus Route Audit

**What it tests**: Checks for baseUrl mismatches and missing pages

**Run**:
```bash
npx tsx test-docusaurus-routes.ts
```

**Expected Output** (if working):
```
✅ Configuration file: FOUND
✅ baseUrl: "/physical_ai_textbook/"
✅ Required pages: ALL PRESENT
  - signup.tsx - EXISTS
  - login.tsx - EXISTS
  - index.tsx - EXISTS
✅ AuthProvider: CONFIGURED
```

**Expected Output** (if failing):
```
❌ signup.tsx - MISSING
⚠️  login.tsx has hard-coded "/signup" (should use baseUrl)
❌ AuthProvider: NOT CONFIGURED
```

**What to look for**:
- ✅ **SUCCESS**: All pages exist, baseUrl is set, AuthProvider configured
- ❌ **Missing pages**: Files not in src/pages/
- ❌ **Hard-coded paths**: Links using `/signup` instead of `${baseUrl}signup`
- ❌ **No AuthProvider**: Root.tsx not wrapping with `<AuthProvider>`

---

## Running All Tests in Sequence

**Quick test all (backend must be running for test 4)**:

```bash
# Terminal 1: Start backend
cd backend
npm run dev

# Terminal 2: Run all tests
cd backend

echo "Test 1: Database Connection"
npx tsx test-db.ts
echo ""

echo "Test 2: Cohere API Key"
npx tsx test-cohere.ts
echo ""

echo "Test 3: JWT Token Inspection"
npx tsx test-jwt.ts
echo ""

echo "Test 4: Auth Route API"
./test-auth.sh
echo ""

echo "Test 5: Docusaurus Routes"
npx tsx test-docusaurus-routes.ts
echo ""

echo "✅ All tests completed!"
```

---

## Interpreting Results

### Scenario 1: Database Test Fails

**Symptoms**:
```
❌ DATABASE TEST FAILED: fetch failed
```

**Diagnosis**: DATABASE_URL is wrong OR Neon is paused

**Fix**:
1. Visit https://console.neon.tech
2. Wake up your database (if paused)
3. Copy fresh DATABASE_URL
4. Update `.env` file
5. Restart backend
6. Re-run test

---

### Scenario 2: Cohere Test Fails (401 Unauthorized)

**Symptoms**:
```
❌ Error: 401 Unauthorized
   Your API key is invalid or expired
```

**Diagnosis**: COHERE_API_KEY is wrong/expired

**Fix**:
1. Go to https://dashboard.cohere.com/api-keys
2. Generate new API key
3. Update `COHERE_API_KEY` in `.env`
4. Re-run test

---

### Scenario 3: Cohere Test Fails (429 Rate Limit)

**Symptoms**:
```
❌ Error: 429 Rate Limit Exceeded
   Trial credits exhausted
```

**Diagnosis**: Trial API quota used up

**Fix**:
1. Upgrade to paid plan at https://dashboard.cohere.com/billing
2. OR use a different API key
3. OR wait for rate limit to reset

---

### Scenario 4: JWT Test Shows Expired Tokens

**Symptoms**:
```
❌ Token Status: EXPIRED
   Expires At (exp): 2025-12-31T...
```

**Diagnosis**: System clock is wrong OR Better-Auth config issue

**Fix**:
1. Check system date: `date`
2. If date is wrong, update system clock
3. Check Better-Auth session config in `src/auth/config.ts`
4. Verify `expiresIn` is set correctly (24 hours = 60 * 60 * 24)

---

### Scenario 5: Auth Route Returns 500

**Symptoms**:
```
HTTP/1.1 500 Internal Server Error
```

**Diagnosis**: Backend is crashing during signup

**What to check**:
1. **Backend terminal logs**: Look for the actual error
2. **Database tables**: Run Test 1 to verify tables exist
3. **Schema mismatch**: Check if `user` table has correct columns
4. **Better-Auth config**: Verify `additionalFields` don't reference non-existent columns

**Common causes**:
- Database tables don't exist (need migrations)
- Better-Auth trying to insert into columns that don't exist
- Foreign key constraint violation

---

### Scenario 6: Docusaurus Routes Show Missing Pages

**Symptoms**:
```
❌ signup.tsx - MISSING
❌ login.tsx - MISSING
```

**Diagnosis**: Page files don't exist in src/pages/

**Fix**:
1. Check if files exist: `ls textbook/src/pages/`
2. If missing, files may have been deleted or moved
3. Restore from our fixed versions in this conversation
4. Or create new signup.tsx and login.tsx

---

### Scenario 7: Hard-coded Paths Found

**Symptoms**:
```
⚠️  signup.tsx has hard-coded "/login" (should use baseUrl)
```

**Diagnosis**: Links not using `siteConfig.baseUrl`

**Fix**:
1. Update signup.tsx:
   ```typescript
   // ❌ WRONG
   <a href="/login">Sign in</a>

   // ✅ CORRECT
   <a href={`${siteConfig.baseUrl}login`}>Sign in</a>
   ```
2. Do same for login.tsx
3. Restart frontend: `npm start`

---

## What to Report Back

After running tests, please share:

1. **Which tests passed/failed**:
   ```
   Test 1 (Database): ✅ PASS
   Test 2 (Cohere): ❌ FAIL - 401 Unauthorized
   Test 3 (JWT): ✅ PASS
   Test 4 (Auth API): ❌ FAIL - HTTP 500
   Test 5 (Docusaurus): ✅ PASS
   ```

2. **Full output of any failing test**:
   - Copy the entire console output
   - Include error messages and stack traces

3. **Backend terminal logs** (if Test 4 fails):
   - What does the backend show when signup is attempted?
   - Any error messages or stack traces?

4. **Environment details**:
   - Node.js version: `node --version`
   - npm version: `npm --version`
   - Operating system

This will help pinpoint the exact root cause!

---

## Expected Test Results (All Working)

If everything is configured correctly, you should see:

```
✅ Test 1: Database connection working, all tables exist
✅ Test 2: Cohere API key valid and working
✅ Test 3: JWT tokens have correct 2026 dates and 24h expiry
✅ Test 4: Signup returns HTTP 201 with token and user data
✅ Test 5: All pages exist, baseUrl configured, AuthProvider present
```

If you get all 5 green checkmarks, your backend is fully functional and the issue is elsewhere (likely frontend connection or CORS).

If any tests fail, focus on fixing those first before testing the full signup flow.

---

**Good luck! Run the tests and share the results!**
