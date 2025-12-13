/**
 * User profile model interface
 * Stores user's background information for personalization
 */
export interface UserProfile {
  id: string;
  user_id: string;
  python_level: PythonLevel;
  ros2_level: ROS2Level;
  gpu_available: GPUAvailability;
  hardware_tier: HardwareTier;
  primary_goal: PrimaryGoal;
  language_preference: LanguagePreference;
  updated_at: Date;
}

/**
 * Profile creation data (for signup)
 */
export interface CreateProfileData {
  python_level: PythonLevel;
  ros2_level: ROS2Level;
  gpu_available: GPUAvailability;
  hardware_tier: HardwareTier;
  primary_goal: PrimaryGoal;
  language_preference?: LanguagePreference;
}

/**
 * Profile update data
 */
export interface UpdateProfileData {
  python_level?: PythonLevel;
  ros2_level?: ROS2Level;
  gpu_available?: GPUAvailability;
  hardware_tier?: HardwareTier;
  primary_goal?: PrimaryGoal;
  language_preference?: LanguagePreference;
}

/**
 * Profile response
 */
export interface ProfileResponse {
  python_level: PythonLevel;
  ros2_level: ROS2Level;
  gpu_available: GPUAvailability;
  hardware_tier: HardwareTier;
  primary_goal: PrimaryGoal;
  language_preference: LanguagePreference;
}

// Enums matching Prisma schema
export enum PythonLevel {
  Beginner = 'Beginner',
  Intermediate = 'Intermediate',
  Expert = 'Expert',
}

export enum ROS2Level {
  None = 'None',
  Beginner = 'Beginner',
  Intermediate = 'Intermediate',
  Expert = 'Expert',
}

export enum GPUAvailability {
  Yes = 'Yes',
  No = 'No',
  Cloud = 'Cloud',
}

export enum HardwareTier {
  Tier1 = 'Tier1',
  Tier2 = 'Tier2',
  Tier3 = 'Tier3',
}

export enum PrimaryGoal {
  Learning = 'Learning',
  Research = 'Research',
  Teaching = 'Teaching',
}

export enum LanguagePreference {
  English = 'English',
  Urdu = 'Urdu',
}
