"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.transformationCache = exports.userProfile = exports.session = exports.verification = exports.account = exports.user = void 0;
const pg_core_1 = require("drizzle-orm/pg-core");
// ============================================================================
// User table (Better-Auth managed)
// ============================================================================
exports.user = (0, pg_core_1.pgTable)('user', {
    id: (0, pg_core_1.uuid)('id').primaryKey().defaultRandom(),
    name: (0, pg_core_1.text)('name').notNull(), // Required by Better-Auth
    email: (0, pg_core_1.varchar)('email', { length: 255 }).notNull().unique(),
    emailVerified: (0, pg_core_1.boolean)('emailVerified').default(false).notNull(), // Required by Better-Auth
    image: (0, pg_core_1.text)('image'), // Optional profile image (Better-Auth)
    password: (0, pg_core_1.text)('password'), // Better-Auth manages password (nullable for OAuth users)
    createdAt: (0, pg_core_1.timestamp)('createdAt').defaultNow().notNull(), // Better-Auth uses camelCase
    updatedAt: (0, pg_core_1.timestamp)('updatedAt').defaultNow().notNull(), // Better-Auth uses camelCase
    created_at: (0, pg_core_1.timestamp)('created_at').defaultNow().notNull(), // Keep for backward compatibility
    last_login: (0, pg_core_1.timestamp)('last_login'),
});
// ============================================================================
// Account table (Better-Auth managed - for OAuth providers)
// ============================================================================
exports.account = (0, pg_core_1.pgTable)('account', {
    id: (0, pg_core_1.uuid)('id').primaryKey().defaultRandom(),
    accountId: (0, pg_core_1.text)('accountId').notNull(), // Provider account ID
    providerId: (0, pg_core_1.text)('providerId').notNull(), // Provider name (e.g., 'google', 'github')
    userId: (0, pg_core_1.uuid)('userId').notNull().references(() => exports.user.id, { onDelete: 'cascade' }),
    accessToken: (0, pg_core_1.text)('accessToken'),
    refreshToken: (0, pg_core_1.text)('refreshToken'),
    idToken: (0, pg_core_1.text)('idToken'),
    expiresAt: (0, pg_core_1.timestamp)('expiresAt'),
    password: (0, pg_core_1.text)('password'), // For email/password provider
    createdAt: (0, pg_core_1.timestamp)('createdAt').defaultNow().notNull(),
    updatedAt: (0, pg_core_1.timestamp)('updatedAt').defaultNow().notNull(),
}, (table) => ({
    userIdIdx: (0, pg_core_1.index)('idx_account_user_id').on(table.userId),
    providerIdx: (0, pg_core_1.index)('idx_account_provider').on(table.providerId, table.accountId),
}));
// ============================================================================
// Verification table (Better-Auth managed - for email verification)
// ============================================================================
exports.verification = (0, pg_core_1.pgTable)('verification', {
    id: (0, pg_core_1.uuid)('id').primaryKey().defaultRandom(),
    identifier: (0, pg_core_1.text)('identifier').notNull(), // Email or phone number
    value: (0, pg_core_1.text)('value').notNull(), // Verification code/token
    expiresAt: (0, pg_core_1.timestamp)('expiresAt').notNull(),
    createdAt: (0, pg_core_1.timestamp)('createdAt').defaultNow().notNull(),
    updatedAt: (0, pg_core_1.timestamp)('updatedAt').defaultNow().notNull(),
}, (table) => ({
    identifierIdx: (0, pg_core_1.index)('idx_verification_identifier').on(table.identifier),
}));
// ============================================================================
// Session table (Better-Auth managed)
// ============================================================================
exports.session = (0, pg_core_1.pgTable)('session', {
    id: (0, pg_core_1.uuid)('id').primaryKey().defaultRandom(), // Better-Auth uses 'id' as primary key
    userId: (0, pg_core_1.uuid)('userId').notNull().references(() => exports.user.id, { onDelete: 'cascade' }), // camelCase
    expiresAt: (0, pg_core_1.timestamp)('expiresAt').notNull(), // camelCase - required by Better-Auth
    token: (0, pg_core_1.text)('token').notNull(), // Session token
    ipAddress: (0, pg_core_1.text)('ipAddress'), // Optional: track IP
    userAgent: (0, pg_core_1.text)('userAgent'), // Optional: track user agent
    createdAt: (0, pg_core_1.timestamp)('createdAt').defaultNow().notNull(), // camelCase
    updatedAt: (0, pg_core_1.timestamp)('updatedAt').defaultNow().notNull(), // camelCase
}, (table) => ({
    userIdIdx: (0, pg_core_1.index)('idx_session_user_id').on(table.userId),
    expiresAtIdx: (0, pg_core_1.index)('idx_session_expires_at').on(table.expiresAt),
    tokenIdx: (0, pg_core_1.index)('idx_session_token').on(table.token),
}));
// ============================================================================
// User profiles table (custom extension)
// ============================================================================
exports.userProfile = (0, pg_core_1.pgTable)('user_profiles', {
    user_id: (0, pg_core_1.uuid)('user_id').primaryKey().references(() => exports.user.id, { onDelete: 'cascade' }),
    // NEW FIELDS - Personalization requirements
    // FR-003: User profile fields for content customization
    software_background: (0, pg_core_1.varchar)('software_background', { length: 20 }).notNull(),
    hardware_experience: (0, pg_core_1.varchar)('hardware_experience', { length: 20 }).notNull(),
    language_preference: (0, pg_core_1.varchar)('language_preference', { length: 10 }).default('English').notNull(),
    // EXISTING FIELDS - Preserve from current schema
    // These fields may already exist in the textbook frontend
    python_level: (0, pg_core_1.varchar)('python_level', { length: 50 }),
    ros2_level: (0, pg_core_1.varchar)('ros2_level', { length: 50 }),
    gpu_available: (0, pg_core_1.varchar)('gpu_available', { length: 10 }),
    hardware_tier: (0, pg_core_1.varchar)('hardware_tier', { length: 50 }),
    primary_goal: (0, pg_core_1.varchar)('primary_goal', { length: 100 }),
    created_at: (0, pg_core_1.timestamp)('created_at').defaultNow().notNull(),
    updated_at: (0, pg_core_1.timestamp)('updated_at').defaultNow().notNull(),
});
// ============================================================================
// Transformation cache table (independent)
// ============================================================================
// FR-036: 5-minute TTL cache for AI-transformed content
exports.transformationCache = (0, pg_core_1.pgTable)('transformation_cache', {
    id: (0, pg_core_1.uuid)('id').primaryKey().defaultRandom(),
    cache_key: (0, pg_core_1.varchar)('cache_key', { length: 64 }).unique().notNull(), // SHA-256 hash
    transformed_content: (0, pg_core_1.text)('transformed_content').notNull(), // Markdown output from LLM
    transformation_metadata: (0, pg_core_1.jsonb)('transformation_metadata'), // { changes_made, complexity_level, model, etc. }
    created_at: (0, pg_core_1.timestamp)('created_at').defaultNow().notNull(),
    expires_at: (0, pg_core_1.timestamp)('expires_at').notNull(), // created_at + 5 minutes
}, (table) => ({
    cacheKeyIdx: (0, pg_core_1.index)('idx_cache_key').on(table.cache_key),
    expiresAtIdx: (0, pg_core_1.index)('idx_expires_at').on(table.expires_at),
}));
