/**
 * Profile Settings component for editing user learning preferences.
 * Implements FR-018 (allow users to update their profile at any time).
 */

import React, { useState, useEffect, FormEvent } from 'react';
import { useAuth } from '../../hooks/useAuth';
import type {
  ExperienceLevel,
  ROSFamiliarity,
  HardwareAccess,
  LearningGoal,
  PreferredLanguage,
  ProfileUpdate,
} from '../../types/auth';
import { updateProfile } from '../../services/authApi';
import { AuthApiError } from '../../services/authApi';
import { SkillLevelBadge } from './SkillLevelBadge';
import styles from './ProfileSettings.module.css';

export function ProfileSettings(): JSX.Element {
  const { user, profile, refreshAuth } = useAuth();
  const [formData, setFormData] = useState<ProfileUpdate>({
    experienceLevel: profile?.experienceLevel || null,
    rosFamiliarity: profile?.rosFamiliarity || null,
    hardwareAccess: profile?.hardwareAccess || null,
    learningGoal: profile?.learningGoal || null,
    preferredLanguage: profile?.preferredLanguage || null,
  });
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  // Update form data when profile changes
  useEffect(() => {
    if (profile) {
      setFormData({
        experienceLevel: profile.experienceLevel,
        rosFamiliarity: profile.rosFamiliarity,
        hardwareAccess: profile.hardwareAccess,
        learningGoal: profile.learningGoal,
        preferredLanguage: profile.preferredLanguage,
      });
    }
  }, [profile]);

  const handleSelectChange = (field: keyof ProfileUpdate, value: string) => {
    setFormData((prev) => ({
      ...prev,
      [field]: value,
    }));
    setSuccess(false);
  };

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError(null);
    setSuccess(false);
    setIsSubmitting(true);

    try {
      await updateProfile(formData);
      await refreshAuth();
      setSuccess(true);
    } catch (err) {
      if (err instanceof AuthApiError) {
        setError(err.message || 'Failed to update profile. Please try again.');
      } else {
        setError('An error occurred. Please try again.');
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!user) {
    return (
      <div className={styles.container}>
        <p>Please log in to edit your profile.</p>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.settings}>
        <div className={styles.header}>
          <h1 className={styles.title}>Profile Settings</h1>
          <p className={styles.subtitle}>
            Update your learning preferences to personalize your experience
          </p>
        </div>

        {/* Skill Level Badge */}
        <div className={styles.skillLevelSection}>
          <h2 className={styles.sectionTitle}>Your Skill Level</h2>
          <p className={styles.sectionDescription}>Based on your profile, you're classified as:</p>
          <SkillLevelBadge showRecalculate size="large" />
          <p className={styles.skillLevelHint}>
            Your skill level is automatically calculated from your programming experience and ROS
            familiarity. Update the fields below and click "Recalculate" to refresh your
            classification.
          </p>
        </div>

        <form onSubmit={handleSubmit} className={styles.form}>
          {/* Programming Experience */}
          <div className={styles.formGroup}>
            <label htmlFor="experienceLevel" className={styles.label}>
              Programming Experience Level
            </label>
            <select
              id="experienceLevel"
              value={formData.experienceLevel || ''}
              onChange={(e) => handleSelectChange('experienceLevel', e.target.value)}
              className={styles.select}
            >
              <option value="">Select your level...</option>
              <option value="beginner">Beginner - New to programming</option>
              <option value="intermediate">Intermediate - Comfortable with basics</option>
              <option value="advanced">Advanced - Experienced programmer</option>
            </select>
          </div>

          {/* ROS Familiarity */}
          <div className={styles.formGroup}>
            <label htmlFor="rosFamiliarity" className={styles.label}>
              ROS Familiarity
            </label>
            <select
              id="rosFamiliarity"
              value={formData.rosFamiliarity || ''}
              onChange={(e) => handleSelectChange('rosFamiliarity', e.target.value)}
              className={styles.select}
            >
              <option value="">Select your familiarity...</option>
              <option value="none">None - Never used ROS</option>
              <option value="basic">Basic - Some exposure to ROS</option>
              <option value="proficient">Proficient - Comfortable with ROS</option>
            </select>
          </div>

          {/* Hardware Access */}
          <div className={styles.formGroup}>
            <label htmlFor="hardwareAccess" className={styles.label}>
              Hardware Access
            </label>
            <select
              id="hardwareAccess"
              value={formData.hardwareAccess || ''}
              onChange={(e) => handleSelectChange('hardwareAccess', e.target.value)}
              className={styles.select}
            >
              <option value="">Select your hardware access...</option>
              <option value="simulation_only">Simulation Only - No physical hardware</option>
              <option value="jetson_kit">Jetson Kit - Jetson development kit</option>
              <option value="full_robot_lab">Full Robot Lab - Complete robotics lab</option>
            </select>
          </div>

          {/* Learning Goal */}
          <div className={styles.formGroup}>
            <label htmlFor="learningGoal" className={styles.label}>
              Learning Goal
            </label>
            <select
              id="learningGoal"
              value={formData.learningGoal || ''}
              onChange={(e) => handleSelectChange('learningGoal', e.target.value)}
              className={styles.select}
            >
              <option value="">Select your goal...</option>
              <option value="career_transition">Career Transition - Switch to robotics</option>
              <option value="academic_research">Academic Research - Pursuing research</option>
              <option value="hobby">Hobby/Personal - Personal interest</option>
            </select>
          </div>

          {/* Preferred Language */}
          <div className={styles.formGroup}>
            <label htmlFor="preferredLanguage" className={styles.label}>
              Preferred Programming Language
            </label>
            <select
              id="preferredLanguage"
              value={formData.preferredLanguage || ''}
              onChange={(e) => handleSelectChange('preferredLanguage', e.target.value)}
              className={styles.select}
            >
              <option value="">Select your preference...</option>
              <option value="python">Python - Show Python examples</option>
              <option value="cpp">C++ - Show C++ examples</option>
              <option value="both">Both - Show both languages</option>
            </select>
          </div>

          {/* Success Message */}
          {success && (
            <div className={styles.successMessage} role="status">
              Profile updated successfully!
            </div>
          )}

          {/* Error Message */}
          {error && (
            <div className={styles.errorMessage} role="alert">
              {error}
            </div>
          )}

          {/* Submit Button */}
          <button type="submit" className={styles.submitButton} disabled={isSubmitting}>
            {isSubmitting ? 'Saving...' : 'Save Changes'}
          </button>
        </form>

        {/* Profile Completion Status */}
        {profile && (
          <div className={styles.completionStatus}>
            <p className={styles.completionText}>
              Profile completion: <strong>{profile.isComplete ? 'Complete' : 'Incomplete'}</strong>
            </p>
            {!profile.isComplete && (
              <p className={styles.completionHint}>
                Complete all fields above to unlock personalized content recommendations.
              </p>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
