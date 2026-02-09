-- Fix password column for Better-Auth compatibility
-- Better-Auth expects a 'password' column (nullable), not 'password_hash'

-- Step 1: Add new 'password' column (nullable, as Better-Auth expects)
ALTER TABLE "user" ADD COLUMN IF NOT EXISTS "password" TEXT;

-- Step 2: Copy existing password hashes to new column (if password_hash exists)
DO $$
BEGIN
  IF EXISTS (
    SELECT 1 FROM information_schema.columns 
    WHERE table_name = 'user' AND column_name = 'password_hash'
  ) THEN
    UPDATE "user" SET "password" = "password_hash" WHERE "password" IS NULL;
  END IF;
END $$;

-- Step 3: Drop old password_hash column (if it exists)
ALTER TABLE "user" DROP COLUMN IF EXISTS "password_hash";

-- Verify the migration
SELECT id, name, email, 
       CASE WHEN "password" IS NOT NULL THEN '***HASHED***' ELSE 'NULL' END AS password_status
FROM "user" 
LIMIT 5;

-- Success message
SELECT 'Password column migration completed!' AS status;
