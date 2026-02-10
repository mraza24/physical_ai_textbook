import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthProvider';
import styles from './AuthForms.module.css';

/**
 * ProfileForm Component
 *
 * Allows authenticated users to update their profile settings:
 * - Software background (Beginner/Intermediate/Expert)
 * - Hardware experience (None/Basic/Advanced)
 * - Language preference (English/Urdu)
 */
export default function ProfileForm() {
  const { user, updateProfile } = useAuth();
  const [softwareBackground, setSoftwareBackground] = useState<string>('Beginner');
  const [hardwareExperience, setHardwareExperience] = useState<string>('None');
  const [languagePreference, setLanguagePreference] = useState<string>('English');
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  // Load current profile on mount
  useEffect(() => {
    if (user?.software_background) {
      setSoftwareBackground(user.software_background);
    }
    if (user?.hardware_experience) {
      setHardwareExperience(user.hardware_experience);
    }
    if (user?.language_preference) {
      setLanguagePreference(user.language_preference);
    }
  }, [user]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setMessage(null);

    try {
      await updateProfile({
        software_background: softwareBackground,
        hardware_experience: hardwareExperience,
        language_preference: languagePreference,
      });

      setMessage({
        type: 'success',
        text: 'Profile updated successfully! Your personalized content will reflect these changes.',
      });
    } catch (error: any) {
      setMessage({
        type: 'error',
        text: error.message || 'Failed to update profile. Please try again.',
      });
    } finally {
      setLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2 className={styles.formTitle}>Update Profile</h2>
      <p className={styles.formSubtitle}>
        Adjust your learning preferences to receive personalized content
      </p>

      {message && (
        <div className={message.type === 'success' ? styles.successMessage : styles.errorMessage}>
          {message.text}
        </div>
      )}

      {/* Software Background */}
      <div className={styles.formGroup}>
        <label htmlFor="softwareBackground" className={styles.label}>
          Software Background
        </label>
        <select
          id="softwareBackground"
          value={softwareBackground}
          onChange={(e) => setSoftwareBackground(e.target.value)}
          className={styles.select}
          required
        >
          <option value="Beginner">Beginner - New to programming and robotics</option>
          <option value="Intermediate">Intermediate - Familiar with Python/C++</option>
          <option value="Expert">Expert - Professional developer</option>
        </select>
      </div>

      {/* Hardware Experience */}
      <div className={styles.formGroup}>
        <label htmlFor="hardwareExperience" className={styles.label}>
          Hardware/Robotics Experience
        </label>
        <select
          id="hardwareExperience"
          value={hardwareExperience}
          onChange={(e) => setHardwareExperience(e.target.value)}
          className={styles.select}
          required
        >
          <option value="None">None - No hands-on experience</option>
          <option value="Basic">Basic - Built simple circuits or robots</option>
          <option value="Advanced">Advanced - Extensive robotics projects</option>
        </select>
      </div>

      {/* Language Preference */}
      <div className={styles.formGroup}>
        <label htmlFor="languagePreference" className={styles.label}>
          Preferred Language
        </label>
        <select
          id="languagePreference"
          value={languagePreference}
          onChange={(e) => setLanguagePreference(e.target.value)}
          className={styles.select}
        >
          <option value="English">English</option>
          <option value="Urdu">Urdu (اردو)</option>
        </select>
      </div>

      {/* Submit Button */}
      <button
        type="submit"
        className={styles.submitButton}
        disabled={loading}
      >
        {loading ? (
          <>
            <span className={styles.spinner}></span>
            Updating...
          </>
        ) : (
          'Save Changes'
        )}
      </button>

      {/* Current Profile Summary */}
      <div className={styles.profileSummary}>
        <h3>Current Profile</h3>
        <ul>
          <li><strong>Software:</strong> {softwareBackground}</li>
          <li><strong>Hardware:</strong> {hardwareExperience}</li>
          <li><strong>Language:</strong> {languagePreference}</li>
        </ul>
      </div>
    </form>
  );
}
