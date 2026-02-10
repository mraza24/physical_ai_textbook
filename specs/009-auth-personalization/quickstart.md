# Quickstart: Authentication & Content Personalization Setup

**Feature**: `009-auth-personalization`
**Estimated Setup Time**: 20-30 minutes
**Date**: 2026-01-01

---

## Prerequisites

Before starting, ensure you have:

- **Node.js 20+**: `node --version` should show v20.x or higher
- **Python 3.11+**: For Claude Agent SDK skills (future)
- **Neon Serverless Postgres Account**: Free tier at [neon.tech](https://neon.tech)
- **Anthropic API Key**: Get from [console.anthropic.com](https://console.anthropic.com)
- **Git**: For version control
- **Text Editor**: VS Code recommended

---

## Step 1: Database Setup (5 minutes)

### 1.1 Create Neon Project

1. Visit [https://neon.tech](https://neon.tech)
2. Sign up or log in
3. Click "Create Project"
4. Name: `physical-ai-textbook-auth`
5. Region: Choose closest to your location
6. Postgres Version: 16 (recommended)
7. Click "Create Project"

### 1.2 Get Connection String

1. In Neon dashboard, click "Connection Details"
2. Copy the connection string (starts with `postgresql://`)
3. **Important**: Note the password (shown only once)

Example connection string:
```
postgresql://username:password@ep-xyz123.us-east-2.aws.neon.tech/neondb?sslmode=require
```

### 1.3 Configure Environment Variables

**Backend .env**:
```bash
cd /mnt/d/Q4_hackathon1/physical_ai_textbook

# Create backend directory if it doesn't exist
mkdir -p backend

# Create .env file
cat > backend/.env <<EOF
DATABASE_URL=postgresql://username:password@ep-xyz123.us-east-2.aws.neon.tech/neondb?sslmode=require
ANTHROPIC_API_KEY=sk-ant-api03-YOUR_KEY_HERE
JWT_SECRET=$(openssl rand -hex 32)
PORT=4000
NODE_ENV=development
EOF
```

**Frontend .env.local**:
```bash
cd textbook

cat > .env.local <<EOF
REACT_APP_API_URL=http://localhost:4000
EOF
```

**Verify**:
```bash
# Backend
cat backend/.env | grep DATABASE_URL
# Should show your Neon connection string

# Frontend
cat textbook/.env.local
# Should show: REACT_APP_API_URL=http://localhost:4000
```

---

## Step 2: Backend Setup (10 minutes)

### 2.1 Initialize Backend Project

```bash
cd backend

# Initialize package.json
npm init -y

# Install core dependencies
npm install express better-auth drizzle-orm @neondatabase/serverless dotenv cors

# Install development dependencies
npm install --save-dev typescript @types/node @types/express ts-node nodemon drizzle-kit

# Initialize TypeScript
npx tsc --init --rootDir src --outDir dist --esModuleInterop --resolveJsonModule --lib es6,dom --module commonjs --allowJs true --noImplicitAny false
```

### 2.2 Configure TypeScript

Edit `backend/tsconfig.json`:
```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "commonjs",
    "lib": ["ES2020"],
    "outDir": "./dist",
    "rootDir": "./src",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "resolveJsonModule": true
  },
  "include": ["src/**/*"],
  "exclude": ["node_modules"]
}
```

### 2.3 Configure Drizzle ORM

Create `backend/drizzle.config.ts`:
```typescript
import type { Config } from 'drizzle-kit';
import * as dotenv from 'dotenv';

dotenv.config();

export default {
  schema: './src/db/schema.ts',
  out: './src/db/migrations',
  driver: 'pg',
  dbCredentials: {
    connectionString: process.env.DATABASE_URL!,
  },
} satisfies Config;
```

### 2.4 Create Database Schema

Create `backend/src/db/schema.ts`:
```typescript
import { pgTable, uuid, varchar, timestamp, boolean, text, jsonb, index } from 'drizzle-orm/pg-core';

// User table (Better-Auth managed)
export const user = pgTable('user', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  password_hash: varchar('password_hash', { length: 255 }).notNull(),
  created_at: timestamp('created_at').defaultNow().notNull(),
  last_login: timestamp('last_login'),
});

// User profiles table (custom extension)
export const userProfile = pgTable('user_profiles', {
  user_id: uuid('user_id').primaryKey().references(() => user.id, { onDelete: 'cascade' }),
  software_background: varchar('software_background', { length: 20 }).notNull(),
  hardware_experience: varchar('hardware_experience', { length: 20 }).notNull(),
  language_preference: varchar('language_preference', { length: 10 }).default('English').notNull(),
  python_level: varchar('python_level', { length: 50 }),
  ros2_level: varchar('ros2_level', { length: 50 }),
  created_at: timestamp('created_at').defaultNow().notNull(),
  updated_at: timestamp('updated_at').defaultNow().notNull(),
});

// Transformation cache table
export const transformationCache = pgTable('transformation_cache', {
  id: uuid('id').primaryKey().defaultRandom(),
  cache_key: varchar('cache_key', { length: 64 }).unique().notNull(),
  transformed_content: text('transformed_content').notNull(),
  transformation_metadata: jsonb('transformation_metadata'),
  created_at: timestamp('created_at').defaultNow().notNull(),
  expires_at: timestamp('expires_at').notNull(),
}, (table) => ({
  cacheKeyIdx: index('idx_cache_key').on(table.cache_key),
  expiresAtIdx: index('idx_expires_at').on(table.expires_at),
}));
```

### 2.5 Create Database Connection

Create `backend/src/db/connection.ts`:
```typescript
import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import * as schema from './schema';

const sql = neon(process.env.DATABASE_URL!);
export const db = drizzle(sql, { schema });
```

### 2.6 Configure Better-Auth

Create `backend/src/auth/config.ts`:
```typescript
import { betterAuth } from 'better-auth';
import { drizzleAdapter } from 'better-auth/adapters/drizzle';
import { db } from '../db/connection';

export const auth = betterAuth({
  database: drizzleAdapter(db, {
    provider: 'pg',
  }),
  secret: process.env.JWT_SECRET!,
  session: {
    expiresIn: 60 * 60 * 24, // 24 hours
    updateAge: 60 * 60, // Refresh every hour
  },
  emailAndPassword: {
    enabled: true,
    bcryptCost: 12,
  },
});
```

### 2.7 Add npm Scripts

Edit `backend/package.json`:
```json
{
  "scripts": {
    "dev": "nodemon --exec ts-node src/index.ts",
    "build": "tsc",
    "start": "node dist/index.js",
    "migrate:generate": "drizzle-kit generate:pg",
    "migrate:push": "drizzle-kit push:pg"
  }
}
```

---

## Step 3: Run Migrations (2 minutes)

### 3.1 Generate Migrations

```bash
cd backend
npm run migrate:generate
```

Expected output:
```
✅ Generated migrations in src/db/migrations/
```

### 3.2 Push to Database

```bash
npm run migrate:push
```

Expected output:
```
✅ Migrations applied successfully
```

### 3.3 Verify Tables Created

Use Neon SQL Editor or:
```bash
# Install psql if not already installed
sudo apt-get install postgresql-client

# Connect to Neon
psql "postgresql://username:password@ep-xyz123.us-east-2.aws.neon.tech/neondb?sslmode=require"

# List tables
\dt

# Expected output:
#  Schema |        Name          | Type  |  Owner
# --------+----------------------+-------+---------
#  public | user                 | table | username
#  public | user_profiles        | table | username
#  public | transformation_cache | table | username
```

---

## Step 4: Frontend Setup (3 minutes)

### 4.1 Install Better-Auth SDK

```bash
cd ../textbook
npm install better-auth
```

### 4.2 Update AuthProvider.tsx

**Before**: Read existing `src/contexts/AuthProvider.tsx` to preserve structure
```bash
cat src/contexts/AuthProvider.tsx | head -20
```

**After**: Extend `UserProfile` interface (lines 12-19):
```typescript
export interface UserProfile {
  // NEW FIELDS
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
  language_preference: 'English' | 'Urdu';

  // EXISTING FIELDS (preserve)
  python_level: string;
  ros2_level: string;
  gpu_available: string;
  hardware_tier: string;
  primary_goal: string;
}
```

---

## Step 5: Start Services (2 minutes)

### 5.1 Start Backend

**Terminal 1**:
```bash
cd backend
npm run dev
```

Expected output:
```
[nodemon] starting `ts-node src/index.ts`
Server running on http://localhost:4000
```

### 5.2 Start Frontend

**Terminal 2**:
```bash
cd textbook
npm run start
```

Expected output:
```
Docusaurus server started on http://localhost:3000
```

---

## Step 6: Test Setup (5 minutes)

### 6.1 Test Signup API

```bash
curl -X POST http://localhost:4000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "testpass123",
    "profile": {
      "software_background": "Beginner",
      "hardware_experience": "None"
    }
  }'
```

**Expected Response**:
```json
{
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "user": {
    "id": "123e4567-e89b-12d3-a456-426614174000",
    "email": "test@example.com",
    "created_at": "2026-01-01T00:00:00.000Z"
  },
  "profile": {
    "software_background": "Beginner",
    "hardware_experience": "None",
    "language_preference": "English"
  }
}
```

### 6.2 Test Login API

```bash
curl -X POST http://localhost:4000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "testpass123",
    "rememberMe": false
  }'
```

### 6.3 Test Profile Retrieval

```bash
# Save token from signup/login response
TOKEN="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."

curl -X GET http://localhost:4000/api/auth/profile \
  -H "Authorization: Bearer $TOKEN"
```

**Expected Response**:
```json
{
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "software_background": "Beginner",
  "hardware_experience": "None",
  "language_preference": "English",
  "python_level": null,
  "ros2_level": null
}
```

### 6.4 Test Frontend Integration

1. Open `http://localhost:3000` in browser
2. Click "Sign Up" (if form exists)
3. Fill email, password, software background (Beginner), hardware experience (None)
4. Submit form
5. Verify redirect to homepage with "Logged In" indicator

---

## Verification Checklist

Run through this checklist to ensure setup is complete:

- [ ] Database connection successful (tables visible in Neon dashboard)
- [ ] Backend server running on port 4000
- [ ] Frontend server running on port 3000
- [ ] User signup returns JWT token
- [ ] User login returns JWT token
- [ ] Profile retrieval works with valid token
- [ ] Profile update persists to database
- [ ] `.env` files contain no placeholder values
- [ ] All dependencies installed without errors
- [ ] No console errors in browser DevTools

---

## Troubleshooting

### Database Connection Fails

**Error**: `Error: connect ECONNREFUSED`

**Solution**:
1. Verify `DATABASE_URL` in `backend/.env`
2. Check Neon project is running (not paused)
3. Test connection:
   ```bash
   psql "$DATABASE_URL" -c "SELECT 1;"
   ```

### JWT Secret Missing

**Error**: `Error: JWT_SECRET is not defined`

**Solution**:
```bash
cd backend
openssl rand -hex 32 | xargs -I {} echo "JWT_SECRET={}" >> .env
```

### Port Already in Use

**Error**: `Error: listen EADDRINUSE: address already in use :::4000`

**Solution**:
```bash
# Find process using port 4000
lsof -i :4000

# Kill process
kill -9 <PID>

# Or change port in backend/.env
echo "PORT=4001" >> .env
```

### TypeScript Errors

**Error**: `Cannot find module '@types/express'`

**Solution**:
```bash
cd backend
rm -rf node_modules package-lock.json
npm install
```

---

## Next Steps

1. ✅ Setup complete
2. ⏭️ Create AI skills in `.claude/skills/`
   - `content-personalizer/skill.md`
   - `urdu-translator/skill.md`
3. ⏭️ Implement chapter action buttons UI
4. ⏭️ Integrate personalization API endpoints
5. ⏭️ Test end-to-end workflow

**Run `/sp.tasks` to generate implementation task breakdown**

---

## Quick Reference

### Start Development Environment

```bash
# Terminal 1 - Backend
cd backend && npm run dev

# Terminal 2 - Frontend
cd textbook && npm run start
```

### Reset Database

```bash
# Drop all tables
psql "$DATABASE_URL" -c "DROP SCHEMA public CASCADE; CREATE SCHEMA public;"

# Re-run migrations
cd backend && npm run migrate:push
```

### View Logs

```bash
# Backend logs
cd backend && npm run dev

# Frontend build logs
cd textbook && npm run build
```

---

**Setup Status**: ✅ READY FOR DEVELOPMENT
**Date**: 2026-01-01
**Next Command**: `/sp.tasks` (generate implementation tasks)
