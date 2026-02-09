-- Add 'name' column to user table (required by Better-Auth)
-- This migration adds the missing 'name' field to the user table

-- Step 1: Add the column (allow NULL temporarily)
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "name" TEXT;

-- Step 2: Update existing users with a default name (email prefix)
UPDATE "user" 
SET "name" = SPLIT_PART(email, '@', 1) 
WHERE "name" IS NULL;

-- Step 3: Make the column NOT NULL now that all rows have values
ALTER TABLE "user" ALTER COLUMN "name" SET NOT NULL;

-- Verify the migration
SELECT id, name, email FROM "user" LIMIT 5;
