"use strict";
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.createUserRateLimiter = createUserRateLimiter;
exports.createTransformationRateLimiter = createTransformationRateLimiter;
exports.createAuthRateLimiter = createAuthRateLimiter;
const express_rate_limit_1 = __importDefault(require("express-rate-limit"));
/**
 * Rate Limiting Service
 * Prevents API abuse by limiting requests per user per time window.
 */
/**
 * Extract user ID from authenticated request
 */
function getUserIdentifier(req) {
    const user = req.user;
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
function createUserRateLimiter(maxRequests = parseInt(process.env.RATE_LIMIT_PER_USER || '10'), windowMinutes = 1) {
    return (0, express_rate_limit_1.default)({
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
        handler: (req, res) => {
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
function createTransformationRateLimiter(maxRequests = 5, windowMinutes = 1) {
    return (0, express_rate_limit_1.default)({
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
        handler: (req, res) => {
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
function createAuthRateLimiter(maxRequests = 5, windowMinutes = 15) {
    return (0, express_rate_limit_1.default)({
        windowMs: windowMinutes * 60 * 1000,
        max: maxRequests,
        // ✅ Fixed: Changed ipKeyGenerator to default
        validate: { default: false },
        keyGenerator: (req) => {
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
