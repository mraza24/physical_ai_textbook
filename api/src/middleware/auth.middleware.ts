import { Request, Response, NextFunction } from 'express';
import authService from '../services/auth.service';

/**
 * Extend Express Request to include user data
 */
export interface AuthenticatedRequest extends Request {
  user?: {
    userId: string;
    email: string;
  };
}

/**
 * Authentication middleware
 * Verifies JWT token from Authorization header
 * Attaches user data to request object if valid
 */
export function authMiddleware(
  req: AuthenticatedRequest,
  res: Response,
  next: NextFunction
): void {
  try {
    // Get token from Authorization header
    const authHeader = req.headers.authorization;

    if (!authHeader || !authHeader.startsWith('Bearer ')) {
      res.status(401).json({
        error: {
          message: 'Authentication required. Please provide a valid token.',
          code: 401,
        },
      });
      return;
    }

    // Extract token (format: "Bearer <token>")
    const token = authHeader.substring(7);

    // Verify token
    const decoded = authService.verifyToken(token);

    // Attach user data to request
    req.user = {
      userId: decoded.userId,
      email: decoded.email,
    };

    next();
  } catch (error) {
    res.status(401).json({
      error: {
        message: error instanceof Error ? error.message : 'Invalid or expired token',
        code: 401,
      },
    });
  }
}

/**
 * Optional authentication middleware
 * Attaches user data if token is present, but doesn't require it
 */
export function optionalAuthMiddleware(
  req: AuthenticatedRequest,
  res: Response,
  next: NextFunction
): void {
  try {
    const authHeader = req.headers.authorization;

    if (authHeader && authHeader.startsWith('Bearer ')) {
      const token = authHeader.substring(7);
      const decoded = authService.verifyToken(token);
      req.user = {
        userId: decoded.userId,
        email: decoded.email,
      };
    }

    next();
  } catch (error) {
    // Ignore errors in optional auth, just proceed without user data
    next();
  }
}
