import crypto from 'crypto';
import { ProfileResponse } from '../models/user-profile';

/**
 * Generate MD5 hash from user profile for cache key
 * Format: python_level|ros2_level|gpu_available|hardware_tier|primary_goal
 */
export function generateProfileHash(profile: ProfileResponse): string {
  const profileString = [
    profile.python_level,
    profile.ros2_level,
    profile.gpu_available,
    profile.hardware_tier,
    profile.primary_goal,
  ].join('|');

  return crypto.createHash('md5').update(profileString).digest('hex');
}

/**
 * Generate cache key for personalized content
 * Format: personalize:{chapterId}:{profileHash}
 */
export function getPersonalizationCacheKey(chapterId: string, profile: ProfileResponse): string {
  const profileHash = generateProfileHash(profile);
  return `personalize:${chapterId}:${profileHash}`;
}
