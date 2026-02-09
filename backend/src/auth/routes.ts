/**
 * Better-Auth Custom Routes
 *
 * T023-T025: Custom signup with profile creation
 *
 * CRITICAL REQUIREMENT:
 * Signup API must capture software_background and hardware_experience
 * from request and save to user_profiles table.
 */

import { Router, Request, Response } from 'express';
import { auth } from './config';
import { db } from '../db/connection';
import { userProfile } from '../db/schema';
import { eq } from 'drizzle-orm';

const router = Router();

/**
 * Custom Signup Request Interface
 *
 * Extends standard Better-Auth signup with profile fields
 */
interface SignupRequest {
  email: string;
  password: string;
  // REQUIRED PERSONALIZATION FIELDS (from spec.md FR-001, FR-002)
  software_background: 'Beginner' | 'Intermediate' | 'Expert';
  hardware_experience: 'None' | 'Basic' | 'Advanced';
  // OPTIONAL FIELDS (from existing schema)
  language_preference?: string;
  python_level?: string;
  ros2_level?: string;
  gpu_available?: string;
  hardware_tier?: string;
  primary_goal?: string;
}

/**
 * Custom Signup Endpoint
 *
 * POST /api/auth/signup
 *
 * Request Body:
 * {
 *   "email": "user@example.com",
 *   "password": "securePassword123",
 *   "software_background": "Beginner",
 *   "hardware_experience": "None",
 *   "language_preference": "English" (optional),
 *   "python_level": "intermediate" (optional),
 *   ... other optional fields
 * }
 *
 * Response (201 Created):
 * {
 *   "user": {
 *     "id": "uuid",
 *     "email": "user@example.com",
 *     "profile": {
 *       "software_background": "Beginner",
 *       "hardware_experience": "None",
 *       ...
 *     }
 *   },
 *   "session": {
 *     "token": "jwt-token",
 *     "expiresAt": "2025-01-02T00:00:00Z"
 *   }
 * }
 *
 * Error Responses:
 * - 400: Missing required fields (email, password, software_background, hardware_experience)
 * - 400: Invalid enum values
 * - 409: Email already exists
 * - 500: Server error
 */
router.post('/signup', async (req: Request, res: Response): Promise<void> => {
  try {
    const {
      email,
      password,
      software_background,
      hardware_experience,
      language_preference = 'English',
      python_level,
      ros2_level,
      gpu_available,
      hardware_tier,
      primary_goal,
    } = req.body as SignupRequest;

    // Validation: Required fields
    if (!email || !password) {
      res.status(400).json({
        error: 'Validation Error',
        message: 'Email and password are required',
        missing_fields: [
          !email ? 'email' : null,
          !password ? 'password' : null,
        ].filter(Boolean),
      });
      return;
    }

    // Validation: CRITICAL PERSONALIZATION FIELDS (FR-001, FR-002)
    if (!software_background || !hardware_experience) {
      res.status(400).json({
        error: 'Validation Error',
        message: 'Profile fields are required for personalization',
        missing_fields: [
          !software_background ? 'software_background' : null,
          !hardware_experience ? 'hardware_experience' : null,
        ].filter(Boolean),
        hint: 'software_background: Beginner | Intermediate | Expert, hardware_experience: None | Basic | Advanced',
      });
      return;
    }

    // Validation: Enum values
    const validSoftwareBackgrounds = ['Beginner', 'Intermediate', 'Expert'];
    const validHardwareExperiences = ['None', 'Basic', 'Advanced'];

    if (!validSoftwareBackgrounds.includes(software_background)) {
      res.status(400).json({
        error: 'Validation Error',
        message: `Invalid software_background. Must be one of: ${validSoftwareBackgrounds.join(', ')}`,
        received: software_background,
      });
      return;
    }

    if (!validHardwareExperiences.includes(hardware_experience)) {
      res.status(400).json({
        error: 'Validation Error',
        message: `Invalid hardware_experience. Must be one of: ${validHardwareExperiences.join(', ')}`,
        received: hardware_experience,
      });
      return;
    }

    // Step 1: Create user with Better-Auth
    // Better-Auth handles password hashing (bcrypt cost 12 from config.ts)
    // NOTE: We only pass email & password to Better-Auth, NOT the profile fields
    // Profile fields are stored separately in user_profiles table to avoid schema conflicts
    const signupResult = await auth.api.signUpEmail({
      body: {
        name: email.split('@')[0], // Use email prefix as name (required by Better-Auth)
        email,
        password,
      },
    });

    if (!signupResult || !signupResult.user) {
      res.status(500).json({
        error: 'Signup Failed',
        message: 'Better-Auth signup failed',
      });
      return;
    }

    const userId = signupResult.user.id;

    // Step 2: Create user_profiles entry (1:1 with user table)
    // CRITICAL: This is where software_background and hardware_experience get persisted
    try {
      await db.insert(userProfile).values({
        user_id: userId,
        software_background,
        hardware_experience,
        language_preference,
        python_level: python_level || null,
        ros2_level: ros2_level || null,
        gpu_available: gpu_available || null,
        hardware_tier: hardware_tier || null,
        primary_goal: primary_goal || null,
        created_at: new Date(),
        updated_at: new Date(),
      });

      console.log(`✅ User profile created for user ${userId}`);
      console.log(`   Software: ${software_background}, Hardware: ${hardware_experience}`);
    } catch (profileError) {
      // Rollback: If profile creation fails, we should delete the user
      // (Better-Auth doesn't provide rollback, so we handle it manually)
      console.error('Profile creation failed, user already created:', profileError);

      res.status(500).json({
        error: 'Profile Creation Failed',
        message: 'User created but profile could not be saved. Contact support.',
        user_id: userId,
      });
      return;
    }

    // Step 3: Return success with user + session
    res.status(201).json({
      user: {
        id: signupResult.user.id,
        email: signupResult.user.email,
        profile: {
          software_background,
          hardware_experience,
          language_preference,
          python_level,
          ros2_level,
          gpu_available,
          hardware_tier,
          primary_goal,
        },
      },
      session: {
        token: signupResult.token,
      },
      message: 'Signup successful! Your profile has been created for personalized content.',
    });

  } catch (error) {
    console.error('Signup error:', error);

    // Handle specific errors
    const errorMessage = error instanceof Error ? error.message : String(error);
    if (errorMessage.includes('duplicate') || errorMessage.includes('unique constraint')) {
      res.status(409).json({
        error: 'Email Already Exists',
        message: 'An account with this email already exists. Try logging in instead.',
      });
      return;
    }

    res.status(500).json({
      error: 'Internal Server Error',
      message: 'Signup failed due to server error',
    });
  }
});

/**
 * Update Profile Endpoint
 *
 * PUT /api/auth/profile
 *
 * Requires: JWT authentication (via requireAuth middleware)
 *
 * Request Body (all fields optional):
 * {
 *   "software_background": "Expert",
 *   "hardware_experience": "Advanced",
 *   "language_preference": "Urdu",
 *   "python_level": "advanced",
 *   ...
 * }
 *
 * Response (200 OK):
 * {
 *   "profile": { updated profile fields },
 *   "message": "Profile updated successfully"
 * }
 */
router.put('/profile', async (req: Request, res: Response): Promise<void> => {
  try {
    // Extract user ID from authenticated request
    // (requireAuth middleware should be applied to this route in index.ts)
    const userId = (req as any).user?.id;

    if (!userId) {
      res.status(401).json({
        error: 'Unauthorized',
        message: 'Authentication required to update profile',
      });
      return;
    }

    const {
      software_background,
      hardware_experience,
      language_preference,
      python_level,
      ros2_level,
      gpu_available,
      hardware_tier,
      primary_goal,
    } = req.body;

    // Validate enum values if provided
    if (software_background) {
      const validSoftwareBackgrounds = ['Beginner', 'Intermediate', 'Expert'];
      if (!validSoftwareBackgrounds.includes(software_background)) {
        res.status(400).json({
          error: 'Validation Error',
          message: `Invalid software_background. Must be one of: ${validSoftwareBackgrounds.join(', ')}`,
        });
        return;
      }
    }

    if (hardware_experience) {
      const validHardwareExperiences = ['None', 'Basic', 'Advanced'];
      if (!validHardwareExperiences.includes(hardware_experience)) {
        res.status(400).json({
          error: 'Validation Error',
          message: `Invalid hardware_experience. Must be one of: ${validHardwareExperiences.join(', ')}`,
        });
        return;
      }
    }

    // Update profile (only fields that were provided)
    const updateData: any = {
      updated_at: new Date(),
    };

    if (software_background !== undefined) updateData.software_background = software_background;
    if (hardware_experience !== undefined) updateData.hardware_experience = hardware_experience;
    if (language_preference !== undefined) updateData.language_preference = language_preference;
    if (python_level !== undefined) updateData.python_level = python_level;
    if (ros2_level !== undefined) updateData.ros2_level = ros2_level;
    if (gpu_available !== undefined) updateData.gpu_available = gpu_available;
    if (hardware_tier !== undefined) updateData.hardware_tier = hardware_tier;
    if (primary_goal !== undefined) updateData.primary_goal = primary_goal;

    await db
      .update(userProfile)
      .set(updateData)
      .where(eq(userProfile.user_id, userId));

    // Fetch updated profile
    const [updatedProfile] = await db
      .select()
      .from(userProfile)
      .where(eq(userProfile.user_id, userId))
      .limit(1);

    res.status(200).json({
      profile: updatedProfile,
      message: 'Profile updated successfully',
    });

  } catch (error) {
    console.error('Profile update error:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: 'Profile update failed',
    });
  }
});

/**
 * Get Profile Endpoint
 *
 * GET /api/auth/profile
 *
 * Requires: JWT authentication
 *
 * Response (200 OK):
 * {
 *   "profile": {
 *     "user_id": "uuid",
 *     "software_background": "Beginner",
 *     "hardware_experience": "None",
 *     ...
 *   }
 * }
 */
router.get('/profile', async (req: Request, res: Response): Promise<void> => {
  try {
    const userId = (req as any).user?.id;

    if (!userId) {
      res.status(401).json({
        error: 'Unauthorized',
        message: 'Authentication required',
      });
      return;
    }

    const [profile] = await db
      .select()
      .from(userProfile)
      .where(eq(userProfile.user_id, userId))
      .limit(1);

    if (!profile) {
      res.status(404).json({
        error: 'Profile Not Found',
        message: 'User profile does not exist',
      });
      return;
    }

    res.status(200).json({
      profile,
    });

  } catch (error) {
    console.error('Get profile error:', error);
    res.status(500).json({
      error: 'Internal Server Error',
      message: 'Failed to retrieve profile',
    });
  }
});

export default router;
