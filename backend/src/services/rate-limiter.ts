import rateLimit from 'express-rate-limit';
import { Request, Response } from 'express';

/**
 * Rate Limiting Service
 * Prevents API abuse by limiting requests per user per time window.
 */

/**
 * Extract user ID from authenticated request
 */
function getUserIdentifier(req: Request): string {
  const user = (req as any).user;
  if (user && user.id) {
    return `user:${user.id}`;
  }

  const forwarded = req.headers['x-forwarded-for'];
  const ip = forwarded
    ? (Array.isArray(forwarded) ? forwarded[0] : forwarded.split(',')[0])
    : req.ip || req.socket.remoteAddress || 'unknown';

  return `ip:${ip}`;
}

/**
 * User-specific rate limiter (FR-035)
 */
export function createUserRateLimiter(
  maxRequests: number = parseInt(process.env.RATE_LIMIT_PER_USER || '10'),
  windowMinutes: number = 1
) {
  return rateLimit({
    windowMs: windowMinutes * 60 * 1000,
    max: maxRequests,
    keyGenerator: getUserIdentifier,
    // ✅ Fixed: Changed ipKeyGenerator to default
    validate: { default: false },
    message: {
      error: 'Too Many Requests',
      message: `Rate limit exceeded. Maximum ${maxRequests} requests per ${windowMinutes} minute(s) allowed.`,
      retryAfter: `${windowMinutes} minute(s)`,
    },
    statusCode: 429,
    standardHeaders: true,
    legacyHeaders: false,
    handler: (req: Request, res: Response) => {
      res.status(429).json({
        error: 'Too Many Requests',
        message: `You have exceeded the rate limit of ${maxRequests} requests per ${windowMinutes} minute(s).`,
        retryAfter: `${windowMinutes} minute(s)`,
        timestamp: new Date().toISOString(),
      });
    },
  });
}

/**
 * AI transformation rate limiter (Expensive operations)
 */
export function createTransformationRateLimiter(
  maxRequests: number = 5,
  windowMinutes: number = 1
) {
  return rateLimit({
    windowMs: windowMinutes * 60 * 1000,
    max: maxRequests,
    keyGenerator: getUserIdentifier,
    // ✅ Fixed: Changed ipKeyGenerator to default
    validate: { default: false }, 
    message: {
      error: 'Transformation Rate Limit Exceeded',
      message: `AI transformation limit exceeded. Maximum ${maxRequests} transformations per ${windowMinutes} minute(s) allowed.`,
      retryAfter: `${windowMinutes} minute(s)`,
      hint: 'Most transformations are cached. Try the same request again after the rate limit resets.',
    },
    statusCode: 429,
    standardHeaders: true,
    legacyHeaders: false,
    handler: (req: Request, res: Response) => {
      res.status(429).json({
        error: 'Transformation Rate Limit Exceeded',
        message: `You have exceeded the AI transformation limit of ${maxRequests} requests per ${windowMinutes} minute(s).`,
        retryAfter: `${windowMinutes} minute(s)`,
        hint: 'Most transformations are cached. Try the same request again after the rate limit resets.',
        timestamp: new Date().toISOString(),
      });
    },
  });
}

/**
 * Auth rate limiter (Prevents brute-force)
 */
export function createAuthRateLimiter(
  maxRequests: number = 5,
  windowMinutes: number = 15
) {
  return rateLimit({
    windowMs: windowMinutes * 60 * 1000,
    max: maxRequests,
    // ✅ Fixed: Changed ipKeyGenerator to default
    validate: { default: false },
    keyGenerator: (req: Request) => {
      const forwarded = req.headers['x-forwarded-for'];
      const ip = forwarded
        ? (Array.isArray(forwarded) ? forwarded[0] : forwarded.split(',')[0])
        : req.ip || req.socket.remoteAddress || 'unknown';
      return ip;
    },
    message: {
      error: 'Authentication Rate Limit Exceeded',
      message: `Too many authentication attempts. Maximum ${maxRequests} attempts per ${windowMinutes} minute(s) allowed.`,
      retryAfter: `${windowMinutes} minute(s)`,
    },
    statusCode: 429,
    standardHeaders: true,
    legacyHeaders: false,
  });
}