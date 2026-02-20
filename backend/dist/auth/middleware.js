"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.requireAuth = requireAuth;
exports.optionalAuth = optionalAuth;
exports.requireRole = requireRole;
exports.requireProfileFields = requireProfileFields;
exports.isSessionExpiringSoon = isSessionExpiringSoon;
exports.getUserId = getUserId;
const config_1 = require("./config");
/**
 * Extract Bearer token from Authorization header
 *
 * @param req - Express request
 * @returns JWT token or null
 */
function extractBearerToken(req) {
    const authHeader = req.headers.authorization;
    if (!authHeader) {
        return null;
    }
    // Check for "Bearer <token>" format
    const parts = authHeader.split(' ');
    if (parts.length !== 2 || parts[0] !== 'Bearer') {
        return null;
    }
    return parts[1];
}
/**
 * Verify JWT token and extract session
 *
 * Uses Better-Auth's session verification
 *
 * @param token - JWT token
 * @returns Session object or null
 */
async function verifyToken(token) {
    try {
        // Better-Auth session verification
        const session = await config_1.auth.api.getSession({
            headers: {
                authorization: `Bearer ${token}`,
            },
        });
        if (!session || !session.user) {
            return null;
        }
        return session;
    }
    catch (error) {
        console.error('Token verification failed:', error);
        return null;
    }
}
/**
 * Require authentication middleware
 *
 * Validates JWT token and attaches user to request.
 * Returns 401 if token is missing, invalid, or expired.
 *
 * @param req - Express request
 * @param res - Express response
 * @param next - Next middleware function
 */
async function requireAuth(req, res, next) {
    try {
        // Extract token from Authorization header
        const token = extractBearerToken(req);
        if (!token) {
            res.status(401).json({
                error: 'Unauthorized',
                message: 'No authentication token provided. Include Bearer token in Authorization header.',
            });
            return;
        }
        // Verify token with Better-Auth
        const session = await verifyToken(token);
        if (!session) {
            res.status(401).json({
                error: 'Unauthorized',
                message: 'Invalid or expired authentication token.',
            });
            return;
        }
        // Attach user and session to request
        req.user = session.user;
        req.session = {
            token,
            expiresAt: new Date(session.session.expiresAt),
        };
        // Continue to next middleware
        next();
    }
    catch (error) {
        console.error('Authentication middleware error:', error);
        res.status(500).json({
            error: 'Internal Server Error',
            message: 'Authentication failed due to server error.',
        });
    }
}
/**
 * Optional authentication middleware
 *
 * Attaches user if token is valid, but doesn't block unauthenticated requests.
 * Useful for endpoints that have different behavior for authenticated vs anonymous users.
 *
 * @param req - Express request
 * @param res - Express response
 * @param next - Next middleware function
 */
async function optionalAuth(req, res, next) {
    try {
        const token = extractBearerToken(req);
        if (token) {
            const session = await verifyToken(token);
            if (session) {
                req.user = session.user;
                req.session = {
                    token,
                    expiresAt: new Date(session.session.expiresAt),
                };
            }
        }
        // Always continue (even if no token or invalid token)
        next();
    }
    catch (error) {
        console.error('Optional auth middleware error:', error);
        // Don't block request on error
        next();
    }
}
/**
 * Require specific user role middleware
 *
 * Checks if authenticated user has required role.
 * Must be used after requireAuth middleware.
 *
 * @param allowedRoles - Array of allowed roles
 * @returns Express middleware function
 */
function requireRole(...allowedRoles) {
    return async (req, res, next) => {
        const authReq = req;
        if (!authReq.user) {
            res.status(401).json({
                error: 'Unauthorized',
                message: 'Authentication required.',
            });
            return;
        }
        // Check if user has required role
        const userRole = authReq.user.role || 'user';
        if (!allowedRoles.includes(userRole)) {
            res.status(403).json({
                error: 'Forbidden',
                message: `Access denied. Required role: ${allowedRoles.join(' or ')}`,
            });
            return;
        }
        next();
    };
}
/**
 * Require user profile fields middleware
 *
 * Ensures user has completed profile setup with required fields.
 * Useful for personalization endpoints that need software_background and hardware_experience.
 *
 * @param requiredFields - Array of required profile fields
 * @returns Express middleware function
 */
function requireProfileFields(...requiredFields) {
    return async (req, res, next) => {
        const authReq = req;
        if (!authReq.user) {
            res.status(401).json({
                error: 'Unauthorized',
                message: 'Authentication required.',
            });
            return;
        }
        // Check if user has all required profile fields
        const missingFields = requiredFields.filter(field => !authReq.user[field]);
        if (missingFields.length > 0) {
            res.status(400).json({
                error: 'Incomplete Profile',
                message: `Profile is missing required fields: ${missingFields.join(', ')}`,
                missingFields,
            });
            return;
        }
        next();
    };
}
/**
 * Check if session is expiring soon (< 1 hour remaining)
 *
 * Can be used to trigger token refresh on client
 *
 * @param req - Authenticated request
 * @returns True if session expires in < 1 hour
 */
function isSessionExpiringSoon(req) {
    const authReq = req;
    if (!authReq.session) {
        return false;
    }
    const now = new Date();
    const expiresAt = authReq.session.expiresAt;
    const oneHourFromNow = new Date(now.getTime() + 60 * 60 * 1000);
    return expiresAt <= oneHourFromNow;
}
/**
 * Extract user ID from request
 *
 * Helper function for logging and rate limiting
 *
 * @param req - Request (potentially authenticated)
 * @returns User ID or 'anonymous'
 */
function getUserId(req) {
    const authReq = req;
    return authReq.user?.id || 'anonymous';
}
