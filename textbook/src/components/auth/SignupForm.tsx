import React, { useState } from 'react';
import { useAuth, UserProfile } from '../../contexts/AuthProvider';
import styles from './AuthForms.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
}

export const SignupForm: React.FC<SignupFormProps> = ({ onSuccess }) => {
  const { signup } = useAuth();

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');

  // REQUIRED PROFILE FIELDS (FR-001, FR-002)
  const [softwareBackground, setSoftwareBackground] = useState<'Beginner' | 'Intermediate' | 'Expert'>('Beginner');
  const [hardwareExperience, setHardwareExperience] = useState<'None' | 'Basic' | 'Advanced'>('None');

  // OPTIONAL PROFILE FIELDS (existing)
  const [pythonLevel, setPythonLevel] = useState<string>('Beginner');
  const [ros2Level, setRos2Level] = useState<string>('None');
  const [gpuAvailable, setGpuAvailable] = useState<string>('No');
  const [hardwareTier, setHardwareTier] = useState<string>('Tier1');
  const [primaryGoal, setPrimaryGoal] = useState<string>('Learning');

  const [error, setError] = useState<string>('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    // Validation
    if (!email || !password || !confirmPassword) {
      setError('All fields are required');
      return;
    }

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    setIsSubmitting(true);

    try {
      const profile: UserProfile = {
        // REQUIRED FIELDS (FR-001, FR-002)
        software_background: softwareBackground,
        hardware_experience: hardwareExperience,
        language_preference: 'English',
        // OPTIONAL FIELDS
        python_level: pythonLevel,
        ros2_level: ros2Level,
        gpu_available: gpuAvailable,
        hardware_tier: hardwareTier,
        primary_goal: primaryGoal,
      };

      await signup(email, password, profile);

      if (onSuccess) {
        onSuccess();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Signup failed');
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.authForm}>
      <h2>Create Account</h2>

      {error && <div className={styles.error}>{error}</div>}

      <div className={styles.formGroup}>
        <label htmlFor="email">Email</label>
        <input
          id="email"
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          required
          placeholder="your.email@example.com"
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="password">Password</label>
        <input
          id="password"
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          required
          placeholder="Min 8 characters, 1 number, 1 special char"
        />
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="confirmPassword">Confirm Password</label>
        <input
          id="confirmPassword"
          type="password"
          value={confirmPassword}
          onChange={(e) => setConfirmPassword(e.target.value)}
          required
          placeholder="Re-enter password"
        />
      </div>

      <h3>Personalization Profile</h3>
      <p className={styles.helpText}>
        Help us adapt content to your skill level
      </p>

      {/* REQUIRED FIELDS (FR-001, FR-002) */}
      <div className={styles.formGroup}>
        <label htmlFor="softwareBackground">
          Software Background <span className={styles.required}>*</span>
        </label>
        <select
          id="softwareBackground"
          value={softwareBackground}
          onChange={(e) => setSoftwareBackground(e.target.value as 'Beginner' | 'Intermediate' | 'Expert')}
          required
          className={styles.neonSelect}
        >
          <option value="Beginner">Beginner (Learning programming fundamentals)</option>
          <option value="Intermediate">Intermediate (1-3 years experience)</option>
          <option value="Expert">Expert (3+ years, production systems)</option>
        </select>
      </div>

      <div className={styles.formGroup}>
        <label htmlFor="hardwareExperience">
          Hardware Experience <span className={styles.required}>*</span>
        </label>
        <select
          id="hardwareExperience"
          value={hardwareExperience}
          onChange={(e) => setHardwareExperience(e.target.value as 'None' | 'Basic' | 'Advanced')}
          required
          className={styles.neonSelect}
        >
          <option value="None">None (First time with robotics hardware)</option>
          <option value="Basic">Basic (Tinkered with Arduino/Raspberry Pi)</option>
          <option value="Advanced">Advanced (Built robots, debugged hardware)</option>
        </select>
      </div>

      <h3>Additional Details (Optional)</h3>
      <p className={styles.helpText}>
        Further customize your experience
      </p>

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
        {isSubmitting ? 'Creating Account...' : 'Sign Up'}
      </button>

      <p className={styles.linkText}>
        Already have an account? <a href="/auth/signin">Sign In</a>
      </p>
    </form>
  );
};
