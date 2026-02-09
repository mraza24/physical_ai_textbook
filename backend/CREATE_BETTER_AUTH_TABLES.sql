-- Create Better-Auth required tables
-- Run this in Neon SQL Editor: https://console.neon.tech

-- ============================================================================
-- Account table (for OAuth and email/password providers)
-- ============================================================================
CREATE TABLE IF NOT EXISTS "account" (
  "id" UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  "accountId" TEXT NOT NULL,
  "providerId" TEXT NOT NULL,
  "userId" UUID NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
  "accessToken" TEXT,
  "refreshToken" TEXT,
  "idToken" TEXT,
  "expiresAt" TIMESTAMP,
  "password" TEXT,
  "createdAt" TIMESTAMP DEFAULT NOW() NOT NULL,
  "updatedAt" TIMESTAMP DEFAULT NOW() NOT NULL
);

-- Create indexes for account table
CREATE INDEX IF NOT EXISTS "idx_account_user_id" ON "account"("userId");
CREATE INDEX IF NOT EXISTS "idx_account_provider" ON "account"("providerId", "accountId");

-- ============================================================================
-- Verification table (for email verification tokens)
-- ============================================================================
CREATE TABLE IF NOT EXISTS "verification" (
  "id" UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  "identifier" TEXT NOT NULL,
  "value" TEXT NOT NULL,
  "expiresAt" TIMESTAMP NOT NULL,
  "createdAt" TIMESTAMP DEFAULT NOW() NOT NULL,
  "updatedAt" TIMESTAMP DEFAULT NOW() NOT NULL
);

-- Create index for verification table
CREATE INDEX IF NOT EXISTS "idx_verification_identifier" ON "verification"("identifier");

-- Verify tables were created
SELECT table_name FROM information_schema.tables 
WHERE table_schema = 'public' 
AND table_name IN ('user', 'account', 'verification', 'session', 'user_profiles')
ORDER BY table_name;

-- Success message
SELECT 'Better-Auth tables created successfully!' AS status;
