/**
 * RFC 5322 Email validation regex
 * Validates email format according to RFC 5322 standard
 */
export const EMAIL_REGEX = /^[a-zA-Z0-9.!#$%&'*+/=?^_`{|}~-]+@[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?(?:\.[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?)*$/;

/**
 * Validate email address
 */
export function isValidEmail(email: string): boolean {
  if (!email || typeof email !== 'string') return false;
  if (email.length > 255) return false;
  return EMAIL_REGEX.test(email);
}

/**
 * Validate password strength
 * Requirements:
 * - Minimum 8 characters
 * - At least one number
 * - At least one special character
 */
export function isValidPassword(password: string): { valid: boolean; errors: string[] } {
  const errors: string[] = [];

  if (!password || typeof password !== 'string') {
    return { valid: false, errors: ['Password is required'] };
  }

  if (password.length < 8) {
    errors.push('Password must be at least 8 characters long');
  }

  if (!/\d/.test(password)) {
    errors.push('Password must contain at least one number');
  }

  if (!/[!@#$%^&*()_+\-=\[\]{};':"\\|,.<>\/?]/.test(password)) {
    errors.push('Password must contain at least one special character');
  }

  return {
    valid: errors.length === 0,
    errors,
  };
}

/**
 * Validate chapter ID format
 * Valid formats: "ch1.1", "ch2.3", "ch3.4", etc.
 */
export function isValidChapterId(chapterId: string): boolean {
  if (!chapterId || typeof chapterId !== 'string') return false;
  return /^ch\d+\.\d+$/.test(chapterId);
}

/**
 * Sanitize string input (remove potential XSS)
 */
export function sanitizeString(input: string): string {
  if (!input || typeof input !== 'string') return '';
  return input
    .replace(/[<>]/g, '')
    .replace(/javascript:/gi, '')
    .trim();
}
