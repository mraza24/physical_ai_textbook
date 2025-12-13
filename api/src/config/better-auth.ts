/**
 * Authentication configuration
 * Using JWT-based authentication for simplicity
 * Better Auth integration can be added later if needed
 */

export const AUTH_CONFIG = {
  // JWT configuration
  jwtSecret: process.env.JWT_SECRET || 'dev-secret-change-in-production',
  jwtExpiresIn: {
    default: '1d', // 1 day for normal login
    rememberMe: '7d', // 7 days with "remember me"
  },

  // Password requirements
  password: {
    minLength: 8,
    requireNumber: true,
    requireSpecialChar: true,
  },

  // Account lockout policy
  lockout: {
    maxFailedAttempts: 3,
    lockDurationMinutes: 15,
  },

  // Session management
  session: {
    cookieName: 'textbook_auth_token',
    sameSite: 'lax' as const,
    secure: process.env.NODE_ENV === 'production',
    httpOnly: true,
  },
};

export default AUTH_CONFIG;
