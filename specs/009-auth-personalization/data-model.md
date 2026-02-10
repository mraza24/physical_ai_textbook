# Data Model: Authentication & Content Personalization

**Feature**: `009-auth-personalization`
**Date**: 2026-01-01
**Database**: Neon Serverless Postgres (PostgreSQL 16)
**ORM**: Drizzle ORM

---

## Entity Relationship Diagram

```
┌─────────────┐       ┌──────────────────┐
│    User     │──────▶│   UserProfile    │
│  (Better-   │  1:1  │   (Custom        │
│   Auth)     │       │   Extension)     │
└─────────────┘       └──────────────────┘
       │
       │ 1:N
       ▼
┌─────────────┐
│   Session   │
│  (Better-   │
│   Auth)     │
└─────────────┘

┌────────────────────────┐
│  TransformationCache   │
│  (Independent - No FK) │
└────────────────────────┘
```

---

## Entities

### 1. User (Managed by Better-Auth)

**Purpose**: Core user authentication entity managed by Better-Auth library.

**Schema** (TypeScript):
```typescript
interface User {
  id: string;              // UUID v4, Primary Key
  email: string;           // Unique, validated email
  password_hash: string;   // bcrypt hash (cost factor 12)
  created_at: Date;        // Account creation timestamp
  last_login: Date | null; // Last successful login (nullable)
}
```

**Drizzle Schema**:
```typescript
// backend/src/db/schema.ts
import { pgTable, uuid, varchar, timestamp } from 'drizzle-orm/pg-core';

export const user = pgTable('user', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  password_hash: varchar('password_hash', { length: 255 }).notNull(),
  created_at: timestamp('created_at').defaultNow().notNull(),
  last_login: timestamp('last_login'),
});
```

**SQL**:
```sql
CREATE TABLE "user" (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  last_login TIMESTAMP
);

CREATE INDEX idx_user_email ON "user"(email);
```

**Validation Rules**:
- Email: RFC 5322 compliant (enforced by Better-Auth)
- Password: Minimum 8 characters (spec FR-003)
- Password Hash: bcrypt with cost factor 12 (spec FR-005)

**Managed By**: Better-Auth (auto-created during `npx better-auth migrate`)

---

### 2. Session (Managed by Better-Auth)

**Purpose**: JWT-based session management for authenticated users.

**Schema** (TypeScript):
```typescript
interface Session {
  session_token: string;   // JWT token (RS256 signed), Primary Key
  user_id: string;         // Foreign Key to User.id
  expires_at: Date;        // Token expiration time
  remember_me: boolean;    // True = 7 days, False = 24 hours
}
```

**Drizzle Schema**:
```typescript
export const session = pgTable('session', {
  session_token: varchar('session_token', { length: 512 }).primaryKey(),
  user_id: uuid('user_id').notNull().references(() => user.id, { onDelete: 'cascade' }),
  expires_at: timestamp('expires_at').notNull(),
  remember_me: boolean('remember_me').default(false).notNull(),
});
```

**SQL**:
```sql
CREATE TABLE session (
  session_token VARCHAR(512) PRIMARY KEY,
  user_id UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  expires_at TIMESTAMP NOT NULL,
  remember_me BOOLEAN DEFAULT FALSE NOT NULL
);

CREATE INDEX idx_session_user_id ON session(user_id);
CREATE INDEX idx_session_expires_at ON session(expires_at);
```

**Token Specifications**:
- Algorithm: RS256 (spec Security requirement)
- Standard Expiration: 24 hours (spec FR-006)
- "Remember Me" Expiration: 7 days (spec FR-006)
- Payload: `{ user_id, email, exp, iat }`

**Cleanup Job**: Expired sessions deleted hourly:
```sql
DELETE FROM session WHERE expires_at < NOW();
```

**Managed By**: Better-Auth (auto-created during migration)

---

### 3. UserProfile (Custom Extension)

**Purpose**: Store user learning preferences and technical background for personalization.

**Schema** (TypeScript):
```typescript
interface UserProfile {
  user_id: string;                    // Primary Key + Foreign Key to User.id
  software_background: 'Beginner' | 'Intermediate' | 'Expert';  // NEW (spec FR-003)
  hardware_experience: 'None' | 'Basic' | 'Advanced';            // NEW (spec FR-003)
  language_preference: 'English' | 'Urdu';                       // NEW (spec FR-003)

  // EXISTING FIELDS (preserve from current schema)
  python_level: string;               // e.g., "Beginner", "Intermediate", "Advanced"
  ros2_level: string;                 // e.g., "None", "Basic", "Expert"
  gpu_available: string;              // e.g., "Yes", "No"
  hardware_tier: string;              // e.g., "Budget", "Mid-range", "High-end"
  primary_goal: string;               // e.g., "Learning", "Research", "Industry"

  created_at: Date;
  updated_at: Date;
}
```

**Drizzle Schema**:
```typescript
export const userProfile = pgTable('user_profiles', {
  user_id: uuid('user_id').primaryKey().references(() => user.id, { onDelete: 'cascade' }),

  // NEW FIELDS
  software_background: varchar('software_background', { length: 20 }).notNull(),
  hardware_experience: varchar('hardware_experience', { length: 20 }).notNull(),
  language_preference: varchar('language_preference', { length: 10 }).default('English').notNull(),

  // EXISTING FIELDS
  python_level: varchar('python_level', { length: 50 }),
  ros2_level: varchar('ros2_level', { length: 50 }),
  gpu_available: varchar('gpu_available', { length: 10 }),
  hardware_tier: varchar('hardware_tier', { length: 50 }),
  primary_goal: varchar('primary_goal', { length: 100 }),

  created_at: timestamp('created_at').defaultNow().notNull(),
  updated_at: timestamp('updated_at').defaultNow().notNull(),
});
```

**SQL**:
```sql
CREATE TABLE user_profiles (
  user_id UUID PRIMARY KEY REFERENCES "user"(id) ON DELETE CASCADE,

  -- NEW FIELDS
  software_background VARCHAR(20) NOT NULL CHECK (software_background IN ('Beginner', 'Intermediate', 'Expert')),
  hardware_experience VARCHAR(20) NOT NULL CHECK (hardware_experience IN ('None', 'Basic', 'Advanced')),
  language_preference VARCHAR(10) DEFAULT 'English' NOT NULL CHECK (language_preference IN ('English', 'Urdu')),

  -- EXISTING FIELDS
  python_level VARCHAR(50),
  ros2_level VARCHAR(50),
  gpu_available VARCHAR(10),
  hardware_tier VARCHAR(50),
  primary_goal VARCHAR(100),

  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  updated_at TIMESTAMP DEFAULT NOW() NOT NULL
);
```

**Validation Rules**:
- `software_background`: ENUM constraint (Beginner, Intermediate, Expert)
- `hardware_experience`: ENUM constraint (None, Basic, Advanced)
- `language_preference`: ENUM constraint (English, Urdu)
- All fields required during signup (spec FR-003)

**Relationship**: One-to-one with User (cascade delete)

**Update Trigger** (auto-update `updated_at`):
```sql
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_user_profiles_updated_at
BEFORE UPDATE ON user_profiles
FOR EACH ROW
EXECUTE FUNCTION update_updated_at_column();
```

---

### 4. TransformationCache (New)

**Purpose**: Temporary storage for AI-transformed chapter content to reduce LLM API costs.

**Schema** (TypeScript):
```typescript
interface TransformationCache {
  id: string;                      // UUID v4, Primary Key
  cache_key: string;               // SHA-256 hash (64 hex chars), Unique
  transformed_content: string;     // Markdown output from LLM (TEXT)
  transformation_metadata: {       // JSONB
    changes_made?: number;         // Number of modifications
    complexity_level?: string;     // "Beginner", "Expert", etc.
    model?: string;                // LLM model used (e.g., "claude-sonnet-4-5")
    cached?: boolean;              // Was this response from cache?
    preserved_terms?: string[];    // (For translation) Technical terms preserved
  };
  created_at: Date;
  expires_at: Date;                // created_at + 5 minutes (spec FR-036)
}
```

**Drizzle Schema**:
```typescript
import { pgTable, uuid, varchar, text, jsonb, timestamp, index } from 'drizzle-orm/pg-core';

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

**SQL**:
```sql
CREATE TABLE transformation_cache (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  cache_key VARCHAR(64) UNIQUE NOT NULL,
  transformed_content TEXT NOT NULL,
  transformation_metadata JSONB,
  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_cache_key ON transformation_cache(cache_key);
CREATE INDEX idx_expires_at ON transformation_cache(expires_at);
```

**Cache Key Generation**:
```typescript
import crypto from 'crypto';

function generateCacheKey(
  chapterPath: string,  // e.g., "/docs/module1/sensors"
  userProfile: { software_background: string; hardware_experience: string },
  transformationType: 'personalize' | 'translate'
): string {
  const data = `${chapterPath}|${userProfile.software_background}|${userProfile.hardware_experience}|${transformationType}`;
  return crypto.createHash('sha256').update(data).digest('hex');
}
```

**Example**:
```typescript
generateCacheKey(
  '/docs/module1/sensors',
  { software_background: 'Beginner', hardware_experience: 'None' },
  'personalize'
);
// Output: "a3f8b2c7e1d4f5a6b8c9d0e1f2a3b4c5d6e7f8a9b0c1d2e3f4a5b6c7d8e9f0a1"
```

**Cleanup Strategy**:
- **Cron Job** (hourly): `DELETE FROM transformation_cache WHERE expires_at < NOW();`
- **Neon Auto-Vacuum**: Automatically reclaims storage
- **Expected Size**: ~500 entries at steady state (50 chapters × 10 user profiles)

**No Foreign Keys**: Cache is independent of users (ephemeral data)

---

## Relationships

### User → UserProfile (1:1)
- **Type**: One-to-one
- **Cascade**: ON DELETE CASCADE (deleting user deletes profile)
- **Enforcement**: Primary key on `user_profiles.user_id`
- **Creation**: Created atomically during signup transaction

### User → Session (1:N)
- **Type**: One-to-many
- **Cascade**: ON DELETE CASCADE (deleting user deletes all sessions)
- **Cleanup**: Expired sessions deleted hourly
- **Average per User**: 1-2 active sessions (desktop + mobile)

### TransformationCache (Independent)
- **No Foreign Keys**: Cache entries are ephemeral and not tied to specific users
- **Rationale**: Allows cache sharing across users with identical profiles
- **Cleanup**: TTL-based expiration (5 minutes)

---

## Migrations

### Migration 001: Better-Auth Schema
**File**: `backend/src/db/migrations/001_better_auth_schema.sql`

```sql
-- Auto-generated by: npx better-auth migrate
CREATE TABLE "user" (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  last_login TIMESTAMP
);

CREATE TABLE session (
  session_token VARCHAR(512) PRIMARY KEY,
  user_id UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  expires_at TIMESTAMP NOT NULL,
  remember_me BOOLEAN DEFAULT FALSE NOT NULL
);

CREATE INDEX idx_user_email ON "user"(email);
CREATE INDEX idx_session_user_id ON session(user_id);
CREATE INDEX idx_session_expires_at ON session(expires_at);
```

**Run Command**: `npx better-auth migrate`

---

### Migration 002: User Profiles Extension
**File**: `backend/src/db/migrations/002_user_profiles_extension.sql`

```sql
CREATE TABLE user_profiles (
  user_id UUID PRIMARY KEY REFERENCES "user"(id) ON DELETE CASCADE,

  -- NEW FIELDS
  software_background VARCHAR(20) NOT NULL CHECK (software_background IN ('Beginner', 'Intermediate', 'Expert')),
  hardware_experience VARCHAR(20) NOT NULL CHECK (hardware_experience IN ('None', 'Basic', 'Advanced')),
  language_preference VARCHAR(10) DEFAULT 'English' NOT NULL CHECK (language_preference IN ('English', 'Urdu')),

  -- EXISTING FIELDS (if migrating from existing table, preserve these)
  python_level VARCHAR(50),
  ros2_level VARCHAR(50),
  gpu_available VARCHAR(10),
  hardware_tier VARCHAR(50),
  primary_goal VARCHAR(100),

  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  updated_at TIMESTAMP DEFAULT NOW() NOT NULL
);

-- Trigger to auto-update updated_at
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_user_profiles_updated_at
BEFORE UPDATE ON user_profiles
FOR EACH ROW
EXECUTE FUNCTION update_updated_at_column();
```

**Run Command**: `npm run drizzle-kit push`

---

### Migration 003: Transformation Cache
**File**: `backend/src/db/migrations/003_transformation_cache.sql`

```sql
CREATE TABLE transformation_cache (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  cache_key VARCHAR(64) UNIQUE NOT NULL,
  transformed_content TEXT NOT NULL,
  transformation_metadata JSONB,
  created_at TIMESTAMP DEFAULT NOW() NOT NULL,
  expires_at TIMESTAMP NOT NULL
);

CREATE INDEX idx_cache_key ON transformation_cache(cache_key);
CREATE INDEX idx_expires_at ON transformation_cache(expires_at);
```

**Run Command**: `npm run drizzle-kit push`

---

## Database Queries

### Common Queries

#### Get User Profile
```typescript
const profile = await db
  .select()
  .from(userProfile)
  .where(eq(userProfile.user_id, userId))
  .limit(1);
```

#### Update Profile
```typescript
await db
  .update(userProfile)
  .set({
    software_background: 'Expert',
    hardware_experience: 'Advanced',
    updated_at: new Date(),
  })
  .where(eq(userProfile.user_id, userId));
```

#### Check Cache
```typescript
const cached = await db
  .select()
  .from(transformationCache)
  .where(
    and(
      eq(transformationCache.cache_key, cacheKey),
      gt(transformationCache.expires_at, new Date())
    )
  )
  .limit(1);
```

#### Insert Cached Content
```typescript
await db.insert(transformationCache).values({
  cache_key: cacheKey,
  transformed_content: content,
  transformation_metadata: { changes_made: 15, model: 'claude-sonnet-4-5' },
  expires_at: new Date(Date.now() + 5 * 60 * 1000), // 5 minutes
});
```

---

## Performance Considerations

### Indexes
- **user.email**: Unique index for fast login lookups (~1ms)
- **session.user_id**: Index for user session queries (~2ms)
- **session.expires_at**: Index for cleanup job (~5ms for batch delete)
- **transformation_cache.cache_key**: Unique index for cache hits (~3ms)
- **transformation_cache.expires_at**: Index for cleanup job (~5ms)

### Query Performance Targets
- User login: < 10ms (email lookup + password verify)
- Profile retrieval: < 5ms (primary key lookup)
- Cache hit: < 5ms (indexed cache_key lookup)
- Cache miss + write: < 100ms (includes LLM API call overhead)

### Storage Estimates
- **User**: ~500 bytes/row × 10,000 users = ~5MB
- **Session**: ~600 bytes/row × 20,000 sessions (2 per user) = ~12MB
- **UserProfile**: ~400 bytes/row × 10,000 profiles = ~4MB
- **TransformationCache**: ~5KB/row × 500 entries = ~2.5MB
- **Total**: ~25MB (well within Neon free tier: 512MB)

---

## Data Model Status

**Status**: ✅ COMPLETE
**Date**: 2026-01-01
**Next Step**: Generate API contracts (OpenAPI specs)

**Dependencies**:
- Better-Auth v2.x
- Drizzle ORM v0.33+
- Neon Serverless Postgres (PostgreSQL 16)
- @neondatabase/serverless v0.9+
