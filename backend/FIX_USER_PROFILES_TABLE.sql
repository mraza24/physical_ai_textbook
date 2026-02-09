-- Fix user_profiles table to remove separate id column
-- Run this in Neon SQL Editor: https://console.neon.tech

-- Drop existing user_profiles table (will recreate with correct schema)
DROP TABLE IF EXISTS "user_profiles" CASCADE;

-- Create user_profiles table with user_id as the only primary key
CREATE TABLE "user_profiles" (
  "user_id" UUID PRIMARY KEY REFERENCES "user"("id") ON DELETE CASCADE,
  
  -- Personalization fields (required)
  "software_background" VARCHAR(20) NOT NULL,
  "hardware_experience" VARCHAR(20) NOT NULL,
  "language_preference" VARCHAR(10) DEFAULT 'English' NOT NULL,
  
  -- Optional profile fields
  "python_level" VARCHAR(50),
  "ros2_level" VARCHAR(50),
  "gpu_available" VARCHAR(10),
  "hardware_tier" VARCHAR(50),
  "primary_goal" VARCHAR(100),
  
  -- Timestamps
  "created_at" TIMESTAMP DEFAULT NOW() NOT NULL,
  "updated_at" TIMESTAMP DEFAULT NOW() NOT NULL
);

-- Create index on user_id (though it's already indexed as PK)
CREATE INDEX IF NOT EXISTS "idx_user_profiles_user_id" ON "user_profiles"("user_id");

-- Verify the table structure
SELECT column_name, data_type, is_nullable, column_default
FROM information_schema.columns 
WHERE table_name = 'user_profiles' 
ORDER BY ordinal_position;

-- Success message
SELECT 'user_profiles table fixed - user_id is now the only primary key!' AS status;
