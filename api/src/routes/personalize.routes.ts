import { Router, Response } from 'express';
import personalizationService from '../services/personalization.service';
import authService from '../services/auth.service';
import { authMiddleware, AuthenticatedRequest } from '../middleware/auth.middleware';
import { personalizeRateLimiter } from '../middleware/rate-limit.middleware';
import { isValidChapterId } from '../utils/validators';

const router = Router();

/**
 * POST /api/personalize
 * Personalize chapter content based on user profile
 * Requires authentication and rate limiting (10 req/min)
 */
router.post(
  '/',
  authMiddleware,
  personalizeRateLimiter,
  async (req: AuthenticatedRequest, res: Response) => {
    try {
      if (!req.user) {
        res.status(401).json({
          error: {
            message: 'Not authenticated',
            code: 401,
          },
        });
        return;
      }

      const { chapterId, originalContent } = req.body;

      // Validate input
      if (!chapterId || !originalContent) {
        res.status(400).json({
          error: {
            message: 'Missing required fields: chapterId, originalContent',
            code: 400,
          },
        });
        return;
      }

      if (!isValidChapterId(chapterId)) {
        res.status(400).json({
          error: {
            message: 'Invalid chapterId format. Expected: ch{module}.{chapter} (e.g., ch1.1)',
            code: 400,
          },
        });
        return;
      }

      if (typeof originalContent !== 'string' || originalContent.length === 0) {
        res.status(400).json({
          error: {
            message: 'originalContent must be a non-empty string',
            code: 400,
          },
        });
        return;
      }

      // Content length limit (10MB from server.ts, but check reasonable size)
      if (originalContent.length > 100000) {
        res.status(400).json({
          error: {
            message: 'Chapter content too large (max 100,000 characters)',
            code: 400,
          },
        });
        return;
      }

      // Get user profile
      const profile = await authService.getProfile(req.user.userId);

      // Personalize content
      const result = await personalizationService.personalizeChapter(
        chapterId,
        originalContent,
        profile
      );

      res.status(200).json({
        personalizedContent: result.personalizedContent,
        cacheHit: result.cacheHit,
        profile: result.profile,
      });
    } catch (error) {
      console.error('Personalization error:', error);

      const message = error instanceof Error ? error.message : 'Personalization failed';
      const statusCode = message.includes('Profile not found') ? 404 : 500;

      res.status(statusCode).json({
        error: {
          message,
          code: statusCode,
        },
      });
    }
  }
);

export default router;
