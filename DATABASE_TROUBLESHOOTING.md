# Database Connection Troubleshooting Guide

**Issue**: Backend crashes with "Database health check failed: TypeError: fetch failed"
**Status**: ✅ RESOLVED with graceful error handling
**Date**: 2026-01-03

---

## What Changed

The backend now starts **successfully even if the database is unavailable**. This prevents the entire server from crashing due to temporary database issues.

### Before (BROKEN)
```
🔍 Testing database connection...
❌ Failed to start server: Error: Database health check failed
[Process exits with code 1 - server doesn't start]
```

### After (FIXED)
```
🔍 Testing database connection...
⚠️  Database connection failed during startup:
Database health check failed: Cannot connect to Neon database.
  Possible causes:
  1. DATABASE_URL is incorrect or malformed
  2. Neon database is paused (wake it up at https://console.neon.tech)
  ...

⚠️  WARNING: Server starting WITHOUT database connection
   - Authentication endpoints will not work
   - Chat endpoints will not work

🚀 UNIFIED BACKEND LIVE ON PORT: 4000
💾 Database Status: ❌ Not Connected

📝 Database Troubleshooting:
   1. Check your .env file has DATABASE_URL set
   2. Verify your Neon database is not paused
   ...
```

---

## Root Cause

The error "TypeError: fetch failed" occurs when:

1. **DATABASE_URL is incorrect or malformed**
   - Missing `sslmode=require` parameter
   - Wrong host/user/password/database name
   - Typo in connection string

2. **Neon database is paused** (most common)
   - Neon pauses databases after ~5 days of inactivity
   - First connection wakes it up but takes 5-10 seconds
   - Solution: Visit https://console.neon.tech and wake it manually

3. **Network connectivity issues**
   - Firewall blocking connections
   - DNS resolution failure
   - Neon service temporarily down

4. **Database doesn't exist**
   - Database was deleted
   - Wrong project selected in Neon console

---

## Solutions

### Solution 1: Check Your DATABASE_URL (Most Common Fix)

**Step 1**: Open your `.env` file
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/backend
cat .env
```

**Step 2**: Verify DATABASE_URL format
```bash
# CORRECT FORMAT:
DATABASE_URL=postgresql://[user]:[password]@[host]/[database]?sslmode=require

# Example (from Neon):
DATABASE_URL=postgresql://neondb_owner:AbCd1234@ep-cool-pond-123456.us-east-2.aws.neon.tech/neondb?sslmode=require
```

**Common Mistakes**:
```bash
# ❌ WRONG: Missing sslmode=require
DATABASE_URL=postgresql://user:pass@host/db

# ❌ WRONG: Wrong scheme (postgres instead of postgresql)
DATABASE_URL=postgres://user:pass@host/db

# ❌ WRONG: Missing credentials
DATABASE_URL=postgresql://host/db

# ✅ CORRECT:
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
```

**Step 3**: Get Fresh DATABASE_URL from Neon
1. Go to https://console.neon.tech
2. Select your project
3. Click "Connection Details" or "Dashboard"
4. Copy the **"Pooled connection"** string (NOT "Direct connection")
5. Paste into your `.env` file

**Step 4**: Verify it works
```bash
# From backend directory
npm run dev

# Should see:
# ✅ Database connected successfully
```

---

### Solution 2: Wake Up Paused Neon Database

Neon automatically pauses databases after inactivity to save resources.

**Symptoms**:
- Backend worked before, now fails
- Error: "fetch failed" or "timeout"
- Neon console shows "Compute: Idle"

**Fix**:
1. Go to https://console.neon.tech
2. Select your project
3. Look for status: "Compute: **Idle**" or "**Paused**"
4. Click on the database or wait for auto-wake
5. Status should change to "Compute: **Active**"
6. Restart your backend: `npm run dev`

**Alternative - Auto-Wake**:
```bash
# First connection wakes the database (takes 5-10 seconds)
# Just wait for the backend to retry (it now has 2 retries with exponential backoff)
```

---

### Solution 3: Test Database Connection Manually

**Using curl**:
```bash
# Test if backend health check endpoint works
curl http://localhost:4000/health

# Expected if DB is working:
{
  "status": "OK",
  "database": {
    "connected": true,
    "serverTime": "2026-01-03T..."
  }
}

# Expected if DB is NOT working:
{
  "status": "ERROR",
  "database": {
    "connected": false
  }
}
```

**Using psql** (if you have PostgreSQL client installed):
```bash
# Copy DATABASE_URL from .env, then:
psql "postgresql://user:pass@ep-xxx.neon.tech/dbname?sslmode=require"

# Should connect and show:
# psql (15.x)
# SSL connection (protocol: TLSv1.3, cipher: TLS_AES_256_GCM_SHA384, bits: 256, compression: off)
# Type "help" for help.
#
# neondb=>

# Test query:
SELECT NOW();
```

---

### Solution 4: Verify Environment Variables Are Loaded

**Check if .env is being read**:
```bash
# From backend directory
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/backend

# Check .env file exists
ls -la .env

# Should show:
# -rw-r--r-- 1 user user 2470 Jan  1 07:26 .env
```

**Verify DATABASE_URL is set**:
```bash
# Add temporary logging to connection.ts
# Or run this in Node.js:
node -e "require('dotenv').config(); console.log(process.env.DATABASE_URL)"

# Should print your DATABASE_URL
# If it prints "undefined", .env is not being loaded
```

**Common .env Loading Issues**:
1. `.env` file in wrong directory (must be in `backend/` folder)
2. `.env` file has wrong permissions
3. Syntax error in `.env` file (no quotes needed around values)

---

## Improved Error Handling

### What We Fixed

**File**: `backend/src/db/connection.ts`

**New Features**:
1. ✅ **Timeout Protection**: 5-second timeout per attempt
2. ✅ **Retry Logic**: 2 retries with exponential backoff (1s, 2s)
3. ✅ **Helpful Error Messages**: Tells you exactly what's wrong

**Example**:
```typescript
export async function healthCheck(timeoutMs: number = 5000, retries: number = 2): Promise<Date> {
  // Tries 3 times total (initial + 2 retries)
  // Waits 1s after first failure, 2s after second failure
  // If Neon is waking up, this gives it time to respond
}
```

**File**: `backend/src/index.ts`

**New Features**:
1. ✅ **Graceful Degradation**: Server starts even if DB fails
2. ✅ **Clear Status**: Shows "💾 Database Status: ❌ Not Connected"
3. ✅ **Troubleshooting Hints**: Prints helpful steps to fix

---

## Error Messages Explained

### Error 1: "Cannot connect to Neon database"
```
Database health check failed: Cannot connect to Neon database.
  Possible causes:
  1. DATABASE_URL is incorrect or malformed
  2. Neon database is paused (wake it up at https://console.neon.tech)
  3. Network connectivity issues
  4. Database does not exist
  Original error: fetch failed
```

**What it means**: The `fetch` API (used by Neon HTTP driver) couldn't reach the database

**Fix**:
1. Check DATABASE_URL in `.env`
2. Visit https://console.neon.tech to wake database
3. Wait 10 seconds and try again

---

### Error 2: "Connection timeout after 5000ms"
```
Database health check failed: Connection timeout after 5000ms.
  The database might be slow to respond or unavailable.
  Original error: Database health check timeout
```

**What it means**: Database is taking too long to respond (waking up from pause, or network is very slow)

**Fix**:
1. Wait a moment and restart backend
2. Check your internet connection
3. Visit Neon console to see if database is waking up

---

### Error 3: "Invalid database response format"
```
Database health check failed: Invalid database response format
```

**What it means**: Query succeeded but returned unexpected data

**Fix**:
1. Check if DATABASE_URL points to correct database
2. Verify database has been migrated (run migrations)

---

## Testing Your Fix

### Test 1: Backend Starts Without Crashing

```bash
cd backend
npm run dev
```

**Expected Output** (even with broken DATABASE_URL):
```
🔍 Testing database connection...
⚠️  Database health check attempt 1 failed, retrying...
⚠️  Database health check attempt 2 failed, retrying...
⚠️  Database connection failed during startup:
Database health check failed: Cannot connect to Neon database.
  ...

⚠️  WARNING: Server starting WITHOUT database connection
   - Authentication endpoints will not work
   - Chat endpoints will not work

======================================================================
🚀 UNIFIED BACKEND LIVE ON PORT: 4000
🌐 CORS ENABLED FOR: http://localhost:3000
📊 Health Check: http://localhost:4000/health
💾 Database Status: ❌ Not Connected
======================================================================
```

✅ **Server is running on port 4000** (doesn't crash)
✅ **Clear error message** tells you what's wrong

---

### Test 2: Health Check Endpoint

```bash
# In another terminal
curl http://localhost:4000/health
```

**If DB is working**:
```json
{
  "status": "OK",
  "database": {
    "connected": true,
    "serverTime": "2026-01-03T12:34:56.789Z"
  }
}
```

**If DB is NOT working**:
```json
{
  "status": "ERROR",
  "database": {
    "connected": false
  }
}
```

---

### Test 3: Retry Logic Works

Watch the console output when starting with a paused database:

```
🔍 Testing database connection...
⚠️  Database health check attempt 1 failed, retrying...
[waits 1 second]
⚠️  Database health check attempt 2 failed, retrying...
[waits 2 seconds]
⚠️  Database connection failed during startup:
...
```

If Neon wakes up during one of the retries, you'll see:
```
🔍 Testing database connection...
⚠️  Database health check attempt 1 failed, retrying...
✅ Database connected successfully
```

---

## How to Fix Your Specific Case

Based on your error, here's the exact steps:

**1. Check if you have a .env file**:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook/backend
ls .env
```

**2. Check if DATABASE_URL is set**:
```bash
grep DATABASE_URL .env
```

**3. If DATABASE_URL looks correct, wake your Neon database**:
- Go to https://console.neon.tech
- Click your project
- Look for "Compute: Idle" or "Compute: Paused"
- Click to wake it up

**4. If you don't have DATABASE_URL, get it from Neon**:
```bash
# Copy .env.example to .env
cp .env.example .env

# Edit .env and add your DATABASE_URL
nano .env  # or use VS Code
```

**5. Restart backend**:
```bash
npm run dev
```

**6. Should see**:
```
✅ Database connected successfully
💾 Database Status: ✅ Connected
```

---

## Quick Reference

### Valid DATABASE_URL Format
```
postgresql://[username]:[password]@[host]:[port]/[database]?sslmode=require

Example:
postgresql://neondb_owner:abc123@ep-cool-pond-12345.us-east-2.aws.neon.tech/neondb?sslmode=require
```

### Environment Variables Checklist
- [✅] DATABASE_URL - Neon Postgres connection string
- [✅] JWT_SECRET - Random string for auth tokens
- [✅] COHERE_API_KEY - From https://dashboard.cohere.com/api-keys
- [⚠️] PORT - Optional (defaults to 4000)
- [⚠️] CORS_ORIGINS - Optional (defaults to localhost:3000)

### Neon Database States
- **Active**: Database is running, connections work immediately
- **Idle**: Database is paused, first connection wakes it (5-10s delay)
- **Starting**: Database is waking up, wait a few seconds
- **Stopped**: Database is fully stopped, must wake manually

---

## Summary

✅ **What Was Fixed**:
1. Backend no longer crashes when database is unavailable
2. Added retry logic (3 attempts with exponential backoff)
3. Added timeout protection (5 seconds per attempt)
4. Clear, actionable error messages
5. Server starts and shows helpful troubleshooting steps

✅ **What You Need to Do**:
1. Make sure `.env` file exists in `backend/` directory
2. Verify DATABASE_URL is set correctly
3. Wake up your Neon database if it's paused
4. Restart backend: `npm run dev`
5. Verify you see: `✅ Database connected successfully`

✅ **Common Fixes**:
- DATABASE_URL wrong format → Copy fresh URL from Neon console
- Database paused → Visit https://console.neon.tech to wake it
- .env not loaded → Check file is in `backend/` directory
- Network issues → Wait a moment and retry

---

**Your backend should now start successfully and give you clear guidance on how to fix any database connection issues!**
