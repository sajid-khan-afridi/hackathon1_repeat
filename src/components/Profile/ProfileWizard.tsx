/**
 * Profile Wizard component for collecting user learning preferences.
 * Implements FR-015, FR-016, FR-017 (profile wizard, skip option, persist answers).
 *
 * Presents 5 questions:
 * 1. Programming experience level
 * 2. ROS familiarity
 * 3. Hardware access
 * 4. Learning goal
 * 5. Preferred code examples
 */

import React, { useState, FormEvent } from 'react';
import type {
  ExperienceLevel,
  ROSFamiliarity,
  HardwareAccess,
  LearningGoal,
  PreferredLanguage,
} from '../../types/auth';
import { createProfile, skipProfile } from '../../services/authApi';
import { AuthApiError } from '../../services/authApi';
import styles from './ProfileWizard.module.css';

interface ProfileWizardProps {
  onComplete: () => void;
  onSkip: () => void;
}

interface ProfileFormData {
  experienceLevel: ExperienceLevel | null;
  rosFamiliarity: ROSFamiliarity | null;
  hardwareAccess: HardwareAccess | null;
  learningGoal: LearningGoal | null;
  preferredLanguage: PreferredLanguage | null;
}

const QUESTIONS = [
  {
    id: 'experienceLevel' as const,
    question: 'What is your programming experience level?',
    options: [
      {
        value: 'beginner' as const,
        label: 'Beginner',
        description: 'New to programming or just starting out',
      },
      {
        value: 'intermediate' as const,
        label: 'Intermediate',
        description: 'Comfortable with basic programming concepts',
      },
      {
        value: 'advanced' as const,
        label: 'Advanced',
        description: 'Experienced programmer with strong fundamentals',
      },
    ],
  },
  {
    id: 'rosFamiliarity' as const,
    question: 'How familiar are you with ROS (Robot Operating System)?',
    options: [
      { value: 'none' as const, label: 'None', description: 'Never used ROS before' },
      { value: 'basic' as const, label: 'Basic', description: 'Some exposure to ROS concepts' },
      {
        value: 'proficient' as const,
        label: 'Proficient',
        description: 'Comfortable working with ROS',
      },
    ],
  },
  {
    id: 'hardwareAccess' as const,
    question: 'What hardware do you have access to?',
    options: [
      {
        value: 'simulation_only' as const,
        label: 'Simulation Only',
        description: 'No physical robot hardware',
      },
      {
        value: 'jetson_kit' as const,
        label: 'Jetson Kit',
        description: 'Have access to Jetson development kit',
      },
      {
        value: 'full_robot_lab' as const,
        label: 'Full Robot Lab',
        description: 'Access to complete robotics lab',
      },
    ],
  },
  {
    id: 'learningGoal' as const,
    question: 'What is your primary learning goal?',
    options: [
      {
        value: 'career_transition' as const,
        label: 'Career Transition',
        description: 'Looking to switch to robotics field',
      },
      {
        value: 'academic_research' as const,
        label: 'Academic Research',
        description: 'Pursuing research in robotics',
      },
      {
        value: 'hobby' as const,
        label: 'Hobby/Personal',
        description: 'Learning for personal interest',
      },
    ],
  },
  {
    id: 'preferredLanguage' as const,
    question: 'Which programming language do you prefer for examples?',
    options: [
      { value: 'python' as const, label: 'Python', description: 'Show Python code examples' },
      { value: 'cpp' as const, label: 'C++', description: 'Show C++ code examples' },
      { value: 'both' as const, label: 'Both', description: 'Show examples in both languages' },
    ],
  },
];

export function ProfileWizard({ onComplete, onSkip }: ProfileWizardProps): JSX.Element {
  const [currentStep, setCurrentStep] = useState(0);
  const [formData, setFormData] = useState<ProfileFormData>({
    experienceLevel: null,
    rosFamiliarity: null,
    hardwareAccess: null,
    learningGoal: null,
    preferredLanguage: null,
  });
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const currentQuestion = QUESTIONS[currentStep];
  const isLastStep = currentStep === QUESTIONS.length - 1;
  const progress = ((currentStep + 1) / QUESTIONS.length) * 100;

  const handleOptionSelect = (value: any) => {
    setFormData((prev) => ({
      ...prev,
      [currentQuestion.id]: value,
    }));
  };

  const handleNext = () => {
    if (currentStep < QUESTIONS.length - 1) {
      setCurrentStep((prev) => prev + 1);
    }
  };

  const handleBack = () => {
    if (currentStep > 0) {
      setCurrentStep((prev) => prev - 1);
      setError(null);
    }
  };

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError(null);

    if (!formData[currentQuestion.id]) {
      setError('Please select an option to continue');
      return;
    }

    if (isLastStep) {
      setIsSubmitting(true);
      try {
        await createProfile({
          experienceLevel: formData.experienceLevel,
          rosFamiliarity: formData.rosFamiliarity,
          hardwareAccess: formData.hardwareAccess,
          learningGoal: formData.learningGoal,
          preferredLanguage: formData.preferredLanguage,
        });
        onComplete();
      } catch (err) {
        if (err instanceof AuthApiError) {
          setError(err.message || 'Failed to save profile. Please try again.');
        } else {
          setError('An error occurred. Please try again.');
        }
      } finally {
        setIsSubmitting(false);
      }
    } else {
      handleNext();
    }
  };

  const handleSkip = async () => {
    setIsSubmitting(true);
    try {
      await skipProfile();
      onSkip();
    } catch (err) {
      if (err instanceof AuthApiError) {
        setError(err.message || 'Failed to skip profile. Please try again.');
      } else {
        setError('An error occurred. Please try again.');
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className={styles.wizardContainer}>
      <div className={styles.wizard}>
        <div className={styles.wizardHeader}>
          <h2 className={styles.wizardTitle}>Complete Your Profile</h2>
          <p className={styles.wizardSubtitle}>
            Help us personalize your learning experience ({currentStep + 1} of {QUESTIONS.length})
          </p>
        </div>

        {/* Progress Bar */}
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: `${progress}%` }} />
        </div>

        <form onSubmit={handleSubmit} className={styles.wizardForm}>
          {/* Question */}
          <div className={styles.questionSection}>
            <h3 className={styles.questionTitle}>{currentQuestion.question}</h3>

            {/* Options */}
            <div className={styles.optionsGrid}>
              {currentQuestion.options.map((option) => {
                const isSelected = formData[currentQuestion.id] === option.value;
                return (
                  <button
                    key={option.value}
                    type="button"
                    onClick={() => handleOptionSelect(option.value)}
                    className={`${styles.optionCard} ${isSelected ? styles.optionCardSelected : ''}`}
                    disabled={isSubmitting}
                  >
                    <div className={styles.optionHeader}>
                      <div className={styles.radioCircle}>
                        {isSelected && <div className={styles.radioInner} />}
                      </div>
                      <span className={styles.optionLabel}>{option.label}</span>
                    </div>
                    <p className={styles.optionDescription}>{option.description}</p>
                  </button>
                );
              })}
            </div>
          </div>

          {/* Error Message */}
          {error && (
            <div className={styles.errorMessage} role="alert">
              {error}
            </div>
          )}

          {/* Navigation Buttons */}
          <div className={styles.navigationButtons}>
            <div>
              {currentStep > 0 && (
                <button
                  type="button"
                  onClick={handleBack}
                  className={styles.backButton}
                  disabled={isSubmitting}
                >
                  Back
                </button>
              )}
            </div>

            <div className={styles.rightButtons}>
              <button
                type="button"
                onClick={handleSkip}
                className={styles.skipButton}
                disabled={isSubmitting}
              >
                Skip for now
              </button>

              <button
                type="submit"
                className={styles.nextButton}
                disabled={isSubmitting || !formData[currentQuestion.id]}
              >
                {isSubmitting ? 'Saving...' : isLastStep ? 'Complete' : 'Next'}
              </button>
            </div>
          </div>
        </form>
      </div>
    </div>
  );
}
