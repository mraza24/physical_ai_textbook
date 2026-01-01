import { pgTable, uuid, varchar, timestamp, boolean, text, jsonb, index } from 'drizzle-orm/pg-core';

// ============================================================================
// User table (Better-Auth managed)
// ============================================================================
export const user = pgTable('user', {
  id: uuid('id').primaryKey().defaultRandom(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  password_hash: varchar('password_hash', { length: 255 }).notNull(),
  created_at: timestamp('created_at').defaultNow().notNull(),
  last_login: timestamp('last_login'),
});

// ============================================================================
// Session table (Better-Auth managed)
// ============================================================================
export const session = pgTable('session', {
  session_token: varchar('session_token', { length: 512 }).primaryKey(),
  user_id: uuid('user_id').notNull().references(() => user.id, { onDelete: 'cascade' }),
  expires_at: timestamp('expires_at').notNull(),
  remember_me: boolean('remember_me').default(false).notNull(),
}, (table) => ({
  userIdIdx: index('idx_session_user_id').on(table.user_id),
  expiresAtIdx: index('idx_session_expires_at').on(table.expires_at),
}));

// ============================================================================
// User profiles table (custom extension)
// ============================================================================
export const userProfile = pgTable('user_profiles', {
  user_id: uuid('user_id').primaryKey().references(() => user.id, { onDelete: 'cascade' }),

  // NEW FIELDS - Personalization requirements
  // FR-003: User profile fields for content customization
  software_background: varchar('software_background', { length: 20 }).notNull(),
  hardware_experience: varchar('hardware_experience', { length: 20 }).notNull(),
  language_preference: varchar('language_preference', { length: 10 }).default('English').notNull(),

  // EXISTING FIELDS - Preserve from current schema
  // These fields may already exist in the textbook frontend
  python_level: varchar('python_level', { length: 50 }),
  ros2_level: varchar('ros2_level', { length: 50 }),
  gpu_available: varchar('gpu_available', { length: 10 }),
  hardware_tier: varchar('hardware_tier', { length: 50 }),
  primary_goal: varchar('primary_goal', { length: 100 }),

  created_at: timestamp('created_at').defaultNow().notNull(),
  updated_at: timestamp('updated_at').defaultNow().notNull(),
});

// ============================================================================
// Transformation cache table (independent)
// ============================================================================
// FR-036: 5-minute TTL cache for AI-transformed content
export const transformationCache = pgTable('transformation_cache', {
  id: uuid('id').primaryKey().defaultRandom(),
  cache_key: varchar('cache_key', { length: 64 }).unique().notNull(), // SHA-256 hash
  transformed_content: text('transformed_content').notNull(), // Markdown output from LLM
  transformation_metadata: jsonb('transformation_metadata'), // { changes_made, complexity_level, model, etc. }
  created_at: timestamp('created_at').defaultNow().notNull(),
  expires_at: timestamp('expires_at').notNull(), // created_at + 5 minutes
}, (table) => ({
  cacheKeyIdx: index('idx_cache_key').on(table.cache_key),
  expiresAtIdx: index('idx_expires_at').on(table.expires_at),
}));

// ============================================================================
// Type exports for use in application code
// ============================================================================
export type User = typeof user.$inferSelect;
export type NewUser = typeof user.$inferInsert;

export type Session = typeof session.$inferSelect;
export type NewSession = typeof session.$inferInsert;

export type UserProfile = typeof userProfile.$inferSelect;
export type NewUserProfile = typeof userProfile.$inferInsert;

export type TransformationCache = typeof transformationCache.$inferSelect;
export type NewTransformationCache = typeof transformationCache.$inferInsert;
