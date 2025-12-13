import { Router, Request, Response } from 'express';
import translationService from '../services/translation.service';
import { translateRateLimiter } from '../middleware/rate-limit.middleware';
import { isValidChapterId } from '../utils/validators';

const router = Router();

/**
 * POST /api/translate
 * Translate chapter content to target language (default: Urdu)
 * Rate limiting: 5 req/min (no auth required)
 */
router.post('/', translateRateLimiter, async (req: Request, res: Response) => {
  try {
    const { chapterId, originalContent, targetLanguage = 'urdu' } = req.body;

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

    // Content length limit
    if (originalContent.length > 100000) {
      res.status(400).json({
        error: {
          message: 'Chapter content too large (max 100,000 characters)',
          code: 400,
        },
      });
      return;
    }

    // Validate target language
    const supportedLanguages = ['urdu'];
    if (!supportedLanguages.includes(targetLanguage)) {
      res.status(400).json({
        error: {
          message: `Unsupported target language: ${targetLanguage}. Supported: ${supportedLanguages.join(', ')}`,
          code: 400,
        },
      });
      return;
    }

    // Translate content
    const result = await translationService.translateChapter(
      chapterId,
      originalContent,
      targetLanguage
    );

    res.status(200).json({
      translatedContent: result.translatedContent,
      cacheHit: result.cacheHit,
      targetLanguage,
      technicalTerms: result.technicalTerms,
    });
  } catch (error) {
    console.error('Translation error:', error);

    const message = error instanceof Error ? error.message : 'Translation failed';
    const statusCode = 500;

    res.status(statusCode).json({
      error: {
        message,
        code: statusCode,
      },
    });
  }
});

export default router;
