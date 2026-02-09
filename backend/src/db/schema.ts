import { pgTable, uuid, varchar, timestamp, boolean, text, jsonb, index } from 'drizzle-orm/pg-core';

// ============================================================================
// User table (Better-Auth managed)
// ============================================================================
export const user = pgTable('user', {
  id: uuid('id').primaryKey().defaultRandom(),
  name: text('name').notNull(), // Required by Better-Auth
  email: varchar('email', { length: 255 }).notNull().unique(),
  emailVerified: boolean('emailVerified').default(false).notNull(), // Required by Better-Auth
  image: text('image'), // Optional profile image (Better-Auth)
  password: text('password'), // Better-Auth manages password (nullable for OAuth users)
  createdAt: timestamp('createdAt').defaultNow().notNull(), // Better-Auth uses camelCase
  updatedAt: timestamp('updatedAt').defaultNow().notNull(), // Better-Auth uses camelCase
  created_at: timestamp('created_at').defaultNow().notNull(), // Keep for backward compatibility
  last_login: timestamp('last_login'),
});

// ============================================================================
// Account table (Better-Auth managed - for OAuth providers)
// ============================================================================
export const account = pgTable('account', {
  id: uuid('id').primaryKey().defaultRandom(),
  accountId: text('accountId').notNull(), // Provider account ID
  providerId: text('providerId').notNull(), // Provider name (e.g., 'google', 'github')
  userId: uuid('userId').notNull().references(() => user.id, { onDelete: 'cascade' }),
  accessToken: text('accessToken'),
  refreshToken: text('refreshToken'),
  idToken: text('idToken'),
  expiresAt: timestamp('expiresAt'),
  password: text('password'), // For email/password provider
  createdAt: timestamp('createdAt').defaultNow().notNull(),
  updatedAt: timestamp('updatedAt').defaultNow().notNull(),
}, (table) => ({
  userIdIdx: index('idx_account_user_id').on(table.userId),
  providerIdx: index('idx_account_provider').on(table.providerId, table.accountId),
}));

// ============================================================================
// Verification table (Better-Auth managed - for email verification)
// ============================================================================
export const verification = pgTable('verification', {
  id: uuid('id').primaryKey().defaultRandom(),
  identifier: text('identifier').notNull(), // Email or phone number
  value: text('value').notNull(), // Verification code/token
  expiresAt: timestamp('expiresAt').notNull(),
  createdAt: timestamp('createdAt').defaultNow().notNull(),
  updatedAt: timestamp('updatedAt').defaultNow().notNull(),
}, (table) => ({
  identifierIdx: index('idx_verification_identifier').on(table.identifier),
}));

// ============================================================================
// Session table (Better-Auth managed)
// ============================================================================
export const session = pgTable('session', {
  id: uuid('id').primaryKey().defaultRandom(), // Better-Auth uses 'id' as primary key
  userId: uuid('userId').notNull().references(() => user.id, { onDelete: 'cascade' }), // camelCase
  expiresAt: timestamp('expiresAt').notNull(), // camelCase - required by Better-Auth
  token: text('token').notNull(), // Session token
  ipAddress: text('ipAddress'), // Optional: track IP
  userAgent: text('userAgent'), // Optional: track user agent
  createdAt: timestamp('createdAt').defaultNow().notNull(), // camelCase
  updatedAt: timestamp('updatedAt').defaultNow().notNull(), // camelCase
}, (table) => ({
  userIdIdx: index('idx_session_user_id').on(table.userId),
  expiresAtIdx: index('idx_session_expires_at').on(table.expiresAt),
  tokenIdx: index('idx_session_token').on(table.token),
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
