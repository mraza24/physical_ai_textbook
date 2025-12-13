import bcrypt from 'bcrypt';
import jwt from 'jsonwebtoken';
import prisma from '../config/database';
import cacheService from './cache.service';
import { isValidEmail, isValidPassword } from '../utils/validators';
import {
  CreateUserData,
  SigninData,
  UserResponse,
} from '../models/user';
import {
  CreateProfileData,
  UpdateProfileData,
  ProfileResponse,
  LanguagePreference,
} from '../models/user-profile';

const BCRYPT_ROUNDS = 10;
const MAX_FAILED_ATTEMPTS = 3;
const LOCK_DURATION_MINUTES = 15;
const JWT_SECRET = process.env.JWT_SECRET || 'dev-secret-change-in-production';

export class AuthService {
  /**
   * Signup: Create new user with profile
   */
  async signup(
    userData: CreateUserData,
    profileData: CreateProfileData
  ): Promise<{ user: UserResponse; profile: ProfileResponse; token: string }> {
    // Validate email
    if (!isValidEmail(userData.email)) {
      throw new Error('Invalid email address');
    }

    // Validate password
    const passwordValidation = isValidPassword(userData.password);
    if (!passwordValidation.valid) {
      throw new Error(passwordValidation.errors.join('. '));
    }

    // Check if email already exists
    const existingUser = await prisma.user.findUnique({
      where: { email: userData.email.toLowerCase() },
    });

    if (existingUser) {
      throw new Error('Email already registered. Log in or use password reset.');
    }

    // Hash password
    const password_hash = await bcrypt.hash(userData.password, BCRYPT_ROUNDS);

    // Create user and profile in transaction
    const result = await prisma.$transaction(async (tx) => {
      const user = await tx.user.create({
        data: {
          email: userData.email.toLowerCase(),
          password_hash,
          created_at: new Date(),
          is_locked: false,
        },
      });

      const profile = await tx.userProfile.create({
        data: {
          user_id: user.id,
          python_level: profileData.python_level,
          ros2_level: profileData.ros2_level,
          gpu_available: profileData.gpu_available,
          hardware_tier: profileData.hardware_tier,
          primary_goal: profileData.primary_goal,
          language_preference: profileData.language_preference || LanguagePreference.English,
        },
      });

      return { user, profile };
    });

    // Generate JWT token (auto-login after signup, 7-day expiration)
    const token = this.generateToken(result.user.id, result.user.email, true);

    // Update last_login
    await prisma.user.update({
      where: { id: result.user.id },
      data: { last_login: new Date() },
    });

    return {
      user: this.sanitizeUser(result.user),
      profile: this.sanitizeProfile(result.profile),
      token,
    };
  }

  /**
   * Signin: Authenticate existing user
   */
  async signin(
    signinData: SigninData
  ): Promise<{ user: UserResponse; profile: ProfileResponse; token: string }> {
    // Find user by email
    const user = await prisma.user.findUnique({
      where: { email: signinData.email.toLowerCase() },
      include: { profile: true },
    });

    if (!user) {
      throw new Error('Invalid credentials');
    }

    // Check if account is locked
    if (user.is_locked && user.lock_expires_at) {
      if (new Date() < user.lock_expires_at) {
        const minutesRemaining = Math.ceil(
          (user.lock_expires_at.getTime() - Date.now()) / 60000
        );
        throw new Error(
          `Account locked due to too many failed attempts. Try again in ${minutesRemaining} minutes.`
        );
      } else {
        // Unlock account if lock period expired
        await prisma.user.update({
          where: { id: user.id },
          data: { is_locked: false, lock_expires_at: null },
        });
      }
    }

    // Verify password
    const passwordMatch = await bcrypt.compare(signinData.password, user.password_hash);

    if (!passwordMatch) {
      // Increment failed attempts (stored in cache)
      const failedKey = `failed_attempts:${user.id}`;
      const failedAttempts = parseInt((await cacheService.get(failedKey)) || '0') + 1;

      if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
        // Lock account
        const lockExpiresAt = new Date(Date.now() + LOCK_DURATION_MINUTES * 60000);
        await prisma.user.update({
          where: { id: user.id },
          data: { is_locked: true, lock_expires_at: lockExpiresAt },
        });
        await cacheService.delete(failedKey);
        throw new Error(
          `Account locked due to too many failed attempts. Try again in ${LOCK_DURATION_MINUTES} minutes.`
        );
      }

      // Store failed attempt count (15 minute TTL)
      await cacheService.set(failedKey, failedAttempts.toString(), 900);
      throw new Error('Invalid credentials');
    }

    // Clear failed attempts on successful login
    await cacheService.delete(`failed_attempts:${user.id}`);

    // Generate JWT token
    const token = this.generateToken(user.id, user.email, signinData.rememberMe || false);

    // Update last_login
    await prisma.user.update({
      where: { id: user.id },
      data: { last_login: new Date() },
    });

    if (!user.profile) {
      throw new Error('User profile not found');
    }

    return {
      user: this.sanitizeUser(user),
      profile: this.sanitizeProfile(user.profile),
      token,
    };
  }

  /**
   * Logout: Invalidate session (client-side token removal)
   */
  async logout(userId: string): Promise<void> {
    // In JWT-based auth, logout is primarily client-side (delete token)
    // Optionally, blacklist token in Redis for added security
    // For now, just log the action
    console.log(`User ${userId} logged out at ${new Date().toISOString()}`);
  }

  /**
   * Get user profile
   */
  async getProfile(userId: string): Promise<ProfileResponse> {
    const profile = await prisma.userProfile.findUnique({
      where: { user_id: userId },
    });

    if (!profile) {
      throw new Error('Profile not found');
    }

    return this.sanitizeProfile(profile);
  }

  /**
   * Update user profile
   */
  async updateProfile(
    userId: string,
    updates: UpdateProfileData
  ): Promise<ProfileResponse> {
    const profile = await prisma.userProfile.update({
      where: { user_id: userId },
      data: {
        ...updates,
        updated_at: new Date(),
      },
    });

    // Invalidate personalization cache for this user
    // Cache keys: personalize:{chapterId}:{profileHash}
    // Since profile changed, invalidate all personalization caches
    await cacheService.invalidatePattern(`personalize:*`);

    return this.sanitizeProfile(profile);
  }

  /**
   * Generate JWT token
   */
  private generateToken(userId: string, email: string, rememberMe: boolean): string {
    const expiresIn = rememberMe ? '7d' : '1d';
    return jwt.sign({ userId, email }, JWT_SECRET, { expiresIn });
  }

  /**
   * Verify JWT token
   */
  verifyToken(token: string): { userId: string; email: string } {
    try {
      const decoded = jwt.verify(token, JWT_SECRET) as { userId: string; email: string };
      return decoded;
    } catch (error) {
      throw new Error('Invalid or expired token');
    }
  }

  /**
   * Sanitize user (remove sensitive data)
   */
  private sanitizeUser(user: any): UserResponse {
    return {
      id: user.id,
      email: user.email,
      created_at: user.created_at,
      last_login: user.last_login,
    };
  }

  /**
   * Sanitize profile
   */
  private sanitizeProfile(profile: any): ProfileResponse {
    return {
      python_level: profile.python_level,
      ros2_level: profile.ros2_level,
      gpu_available: profile.gpu_available,
      hardware_tier: profile.hardware_tier,
      primary_goal: profile.primary_goal,
      language_preference: profile.language_preference,
    };
  }
}

// Export singleton instance
export default new AuthService();
