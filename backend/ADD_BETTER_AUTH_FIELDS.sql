-- Add Better-Auth required fields to user table
-- Run this in Neon SQL Editor: https://console.neon.tech

-- Step 1: Add emailVerified column (required by Better-Auth)
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "emailVerified" BOOLEAN DEFAULT false NOT NULL;

-- Step 2: Add image column (optional profile picture)
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "image" TEXT;

-- Step 3: Add Better-Auth timestamp columns (camelCase)
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "createdAt" TIMESTAMP DEFAULT NOW() NOT NULL;
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "updatedAt" TIMESTAMP DEFAULT NOW() NOT NULL;

-- Step 4: Sync existing timestamps for existing users
UPDATE "user" 
SET "createdAt" = COALESCE("created_at", NOW()),
    "updatedAt" = COALESCE("created_at", NOW())
WHERE "createdAt" IS NULL OR "updatedAt" IS NULL;

-- Verify the migration
SELECT id, name, email, "emailVerified", "createdAt", "updatedAt" FROM "user" LIMIT 5;

-- Success message
SELECT 'Migration completed successfully!' AS status;
