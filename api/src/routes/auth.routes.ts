import { Router, Response } from 'express';
import authService from '../services/auth.service';
import { authMiddleware, AuthenticatedRequest } from '../middleware/auth.middleware';
import { CreateUserData } from '../models/user';
import { CreateProfileData, UpdateProfileData } from '../models/user-profile';

const router = Router();

/**
 * POST /api/auth/signup
 * Create new user account with profile
 */
router.post('/signup', async (req: AuthenticatedRequest, res: Response) => {
  try {
    const { email, password, profile } = req.body;

    // Validate request body
    if (!email || !password || !profile) {
      res.status(400).json({
        error: {
          message: 'Missing required fields: email, password, profile',
          code: 400,
        },
      });
      return;
    }

    // Validate profile fields
    const requiredProfileFields = [
      'python_level',
      'ros2_level',
      'gpu_available',
      'hardware_tier',
      'primary_goal',
    ];

    for (const field of requiredProfileFields) {
      if (!profile[field]) {
        res.status(400).json({
          error: {
            message: `Missing required profile field: ${field}`,
            code: 400,
          },
        });
        return;
      }
    }

    const userData: CreateUserData = { email, password };
    const profileData: CreateProfileData = profile;

    const result = await authService.signup(userData, profileData);

    res.status(201).json({
      user: result.user,
      profile: result.profile,
      token: result.token,
    });
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Signup failed';
    const statusCode = message.includes('already registered') ? 409 : 400;

    res.status(statusCode).json({
      error: {
        message,
        code: statusCode,
      },
    });
  }
});

/**
 * POST /api/auth/signin
 * Sign in existing user
 */
router.post('/signin', async (req: AuthenticatedRequest, res: Response) => {
  try {
    const { email, password, rememberMe } = req.body;

    if (!email || !password) {
      res.status(400).json({
        error: {
          message: 'Missing required fields: email, password',
          code: 400,
        },
      });
      return;
    }

    const result = await authService.signin({ email, password, rememberMe });

    res.status(200).json({
      user: result.user,
      profile: result.profile,
      token: result.token,
    });
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Signin failed';
    const statusCode = message.includes('locked') ? 423 : 401;

    res.status(statusCode).json({
      error: {
        message,
        code: statusCode,
        ...(statusCode === 423 && { lock_expires_at: new Date(Date.now() + 15 * 60000) }),
      },
    });
  }
});

/**
 * POST /api/auth/logout
 * Log out current user
 */
router.post('/logout', authMiddleware, async (req: AuthenticatedRequest, res: Response) => {
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

    await authService.logout(req.user.userId);

    res.status(200).json({
      message: 'Logout successful',
    });
  } catch (error) {
    res.status(500).json({
      error: {
        message: 'Logout failed',
        code: 500,
      },
    });
  }
});

/**
 * GET /api/auth/profile
 * Get current user's profile
 */
router.get('/profile', authMiddleware, async (req: AuthenticatedRequest, res: Response) => {
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

    const profile = await authService.getProfile(req.user.userId);

    res.status(200).json({ profile });
  } catch (error) {
    res.status(404).json({
      error: {
        message: error instanceof Error ? error.message : 'Profile not found',
        code: 404,
      },
    });
  }
});

/**
 * PUT /api/auth/profile
 * Update current user's profile
 */
router.put('/profile', authMiddleware, async (req: AuthenticatedRequest, res: Response) => {
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

    const updates: UpdateProfileData = req.body;

    const profile = await authService.updateProfile(req.user.userId, updates);

    res.status(200).json({ profile });
  } catch (error) {
    res.status(400).json({
      error: {
        message: error instanceof Error ? error.message : 'Profile update failed',
        code: 400,
      },
    });
  }
});

export default router;
