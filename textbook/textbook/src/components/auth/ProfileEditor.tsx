import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthProvider';
import styles from './AuthForms.module.css';

export const ProfileEditor: React.FC = () => {
  const { profile, updateProfile } = useAuth();

  const [pythonLevel, setPythonLevel] = useState<string>('Beginner');
  const [ros2Level, setRos2Level] = useState<string>('None');
  const [gpuAvailable, setGpuAvailable] = useState<string>('No');
  const [hardwareTier, setHardwareTier] = useState<string>('Tier1');
  const [primaryGoal, setPrimaryGoal] = useState<string>('Learning');

  const [successMessage, setSuccessMessage] = useState<string>('');
  const [error, setError] = useState<string>('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Load profile data on mount
  useEffect(() => {
    if (profile) {
      setPythonLevel(profile.python_level);
      setRos2Level(profile.ros2_level);
      setGpuAvailable(profile.gpu_available);
      setHardwareTier(profile.hardware_tier);
      setPrimaryGoal(profile.primary_goal);
    }
  }, [profile]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccessMessage('');
    setIsSubmitting(true);

    try {
      await updateProfile({
        python_level: pythonLevel,
        ros2_level: ros2Level,
        gpu_available: gpuAvailable,
        hardware_tier: hardwareTier,
        primary_goal: primaryGoal,
      });

      setSuccessMessage('Profile updated successfully! Personalized content will refresh.');

      // Clear success message after 3 seconds
      setTimeout(() => setSuccessMessage(''), 3000);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Profile update failed');
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!profile) {
    return <div>Loading profile...</div>;
  }

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2>Edit Profile</h2>
      <p className={styles.helpText}>
        Update your background to personalize learning content
      </p>

      {successMessage && <div className={styles.success}>{successMessage}</div>}
      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.formGroup}>
        <label htmlFor="pythonLevel">Python Experience</label>
        <select
          id="pythonLevel"
          value={pythonLevel}
          onChange={(e) => setPythonLevel(e.target.value)}
          required
        >
          <option value="Beginner">Beginner (New to Python)</option>
          <option value="Intermediate">Intermediate (1-2 years)</option>
          <option value="Expert">Expert (3+ years)</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="ros2Level">ROS 2 Experience</label>
        <select
          id="ros2Level"
          value={ros2Level}
          onChange={(e) => setRos2Level(e.target.value)}
          required
        >
          <option value="None">None (First time)</option>
          <option value="Beginner">Beginner (Basic concepts)</option>
          <option value="Intermediate">Intermediate (Built projects)</option>
          <option value="Expert">Expert (Production systems)</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="gpuAvailable">GPU Access</label>
        <select
          id="gpuAvailable"
          value={gpuAvailable}
          onChange={(e) => setGpuAvailable(e.target.value)}
          required
        >
          <option value="Yes">Yes (RTX 3060+ or similar)</option>
          <option value="No">No (CPU only)</option>
          <option value="Cloud">Cloud (AWS, GCP, Colab)</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="hardwareTier">Hardware Tier</label>
        <select
          id="hardwareTier"
          value={hardwareTier}
          onChange={(e) => setHardwareTier(e.target.value)}
          required
        >
          <option value="Tier1">Tier 1 (Jetson Nano, Pi 4)</option>
          <option value="Tier2">Tier 2 (Jetson Orin Nano, RTX 3060)</option>
          <option value="Tier3">Tier 3 (Jetson AGX, RTX 4070+)</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="primaryGoal">Primary Goal</label>
        <select
          id="primaryGoal"
          value={primaryGoal}
          onChange={(e) => setPrimaryGoal(e.target.value)}
          required
        >
          <option value="Learning">Learning (Self-study)</option>
          <option value="Research">Research (Academia/Industry)</option>
          <option value="Teaching">Teaching (Instructor/TA)</option>
        </select>
      </div>

      <button type="submit" disabled={isSubmitting} className={styles.submitButton}>
        {isSubmitting ? 'Saving...' : 'Save Changes'}
      </button>
    </form>
  );
};
