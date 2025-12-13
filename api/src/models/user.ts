/**
 * User model interface
 * Represents a registered user (student or instructor) with authentication credentials
 */
export interface User {
  id: string;
  email: string;
  password_hash: string;
  created_at: Date;
  last_login: Date | null;
  is_locked: boolean;
  lock_expires_at: Date | null;
}

/**
 * User creation data (for signup)
 */
export interface CreateUserData {
  email: string;
  password: string; // Plain password (will be hashed)
}

/**
 * User signin data
 */
export interface SigninData {
  email: string;
  password: string;
  rememberMe?: boolean;
}

/**
 * User response (without sensitive data)
 */
export interface UserResponse {
  id: string;
  email: string;
  created_at: Date;
  last_login: Date | null;
}
