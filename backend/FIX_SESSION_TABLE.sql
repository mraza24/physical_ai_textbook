-- Fix session table to match Better-Auth expectations
-- Run this in Neon SQL Editor: https://console.neon.tech

-- Drop existing session table (will recreate with correct schema)
DROP TABLE IF EXISTS "session" CASCADE;

-- Create session table with Better-Auth compatible schema
CREATE TABLE "session" (
  "id" UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  "userId" UUID NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
  "expiresAt" TIMESTAMP NOT NULL,
  "token" TEXT NOT NULL,
  "ipAddress" TEXT,
  "userAgent" TEXT,
  "createdAt" TIMESTAMP DEFAULT NOW() NOT NULL,
  "updatedAt" TIMESTAMP DEFAULT NOW() NOT NULL
);

-- Create indexes
CREATE INDEX IF NOT EXISTS "idx_session_user_id" ON "session"("userId");
CREATE INDEX IF NOT EXISTS "idx_session_expires_at" ON "session"("expiresAt");
CREATE INDEX IF NOT EXISTS "idx_session_token" ON "session"("token");

-- Verify the session table structure
SELECT column_name, data_type, is_nullable 
FROM information_schema.columns 
WHERE table_name = 'session' 
ORDER BY ordinal_position;

-- Success message
SELECT 'Session table fixed successfully!' AS status;
