#!/usr/bin/env node

import { strict as assert } from 'assert';
import { readFileSync, writeFileSync } from 'fs';
import ContentAdapter from './index.js';

/**
 * Test suite for Content Adapter skill
 */
class ContentAdapterTests {
  constructor() {
    this.adapter = new ContentAdapter();
    this.testResults = [];
    this.sampleMDX = this.createSampleMDX();
    this.userProfiles = this.createTestProfiles();
  }

  /**
   * Run all tests
   */
  async runAllTests() {
    console.log('🧪 Running Content Adapter Tests...\n');

    await this.testExperienceLevelAdaptation();
    await this.testROSAdaptation();
    await this.testHardwareAdaptation();
    await this.testLanguageAdaptation();
    await this.testCacheFunctionality();
    await this.testMetadataGeneration();

    this.printResults();
  }

  /**
   * Test experience level adaptations
   */
  async testExperienceLevelAdaptation() {
    console.log('Testing Experience Level Adaptations...');

    const tests = [
      {
        name: 'Beginner simplification',
        profile: { experience_level: 'beginner' },
        expectedAdaptations: ['beginner-simplification'],
        expectedDifficulty: '🟢 Beginner-Friendly'
      },
      {
        name: 'Intermediate optimizations',
        profile: { experience_level: 'intermediate' },
        expectedAdaptations: ['intermediate-optimizations'],
        expectedDifficulty: '🟡 Intermediate'
      },
      {
        name: 'Advanced topics',
        profile: { experience_level: 'advanced' },
        expectedAdaptations: ['advanced-topics'],
        expectedDifficulty: '🔴 Advanced'
      }
    ];

    for (const test of tests) {
      const result = await this.adapter.adaptContent(this.sampleMDX, test.profile);

      assert(
        result.metadata.adaptations_applied.includes(test.expectedAdaptations[0]),
        `Expected ${test.expectedAdaptations[0]} in adaptations`
      );

      assert(
        result.metadata.difficulty === test.expectedDifficulty,
        `Expected difficulty "${test.expectedDifficulty}" for ${test.name}`
      );

      this.testResults.push({ test: test.name, status: 'PASS' });
    }
  }

  /**
   * Test ROS familiarity adaptations
   */
  async testROSAdaptation() {
    console.log('Testing ROS Adaptations...');

    const tests = [
      {
        name: 'No ROS - adds basics',
        profile: { ros_familiarity: 'none' },
        expected: 'ros-basics'
      },
      {
        name: 'Basic ROS - adds patterns',
        profile: { ros_familiarity: 'basic' },
        expected: 'ros-patterns'
      },
      {
        name: 'Proficient ROS - adds advanced',
        profile: { ros_familiarity: 'proficient' },
        expected: 'advanced-ros'
      }
    ];

    for (const test of tests) {
      const result = await this.adapter.adaptContent(this.sampleMDX, test.profile);

      assert(
        result.metadata.adaptations_applied.includes(test.expected),
        `Expected ${test.expected} adaptation for ${test.name}`
      );

      this.testResults.push({ test: test.name, status: 'PASS' });
    }
  }

  /**
   * Test hardware access adaptations
   */
  async testHardwareAdaptation() {
    console.log('Testing Hardware Access Adaptations...');

    const tests = [
      {
        name: 'Simulation only',
        profile: { hardware_access: 'simulation_only' },
        expected: 'simulation-focus'
      },
      {
        name: 'Partial lab',
        profile: { hardware_access: 'partial_lab' },
        expected: 'hybrid-examples'
      },
      {
        name: 'Full lab',
        profile: { hardware_access: 'full_lab' },
        expected: 'hardware-deployment'
      }
    ];

    for (const test of tests) {
      const result = await this.adapter.adaptContent(this.sampleMDX, test.profile);

      assert(
        result.metadata.adaptations_applied.includes(test.expected),
        `Expected ${test.expected} adaptation for ${test.name}`
      );

      this.testResults.push({ test: test.name, status: 'PASS' });
    }
  }

  /**
   * Test language preference adaptations
   */
  async testLanguageAdaptation() {
    console.log('Testing Language Adaptations...');

    const result = await this.adapter.adaptContent(this.sampleMDX, {
      preferred_language: 'cpp'
    });

    assert(
      result.metadata.adaptations_applied.includes('language-cpp'),
      'Expected language-cpp adaptation'
    );

    this.testResults.push({ test: 'Language adaptation (C++)', status: 'PASS' });
  }

  /**
   * Test caching functionality
   */
  async testCacheFunctionality() {
    console.log('Testing Cache Functionality...');

    const profile = this.userProfiles.intermediate;
    const startTime = Date.now();

    // First call - should populate cache
    const result1 = await this.adapter.adaptContent(this.sampleMDX, profile);
    const firstCallTime = Date.now() - startTime;

    const secondStartTime = Date.now();
    // Second call - should use cache
    const result2 = await this.adapter.adaptContent(this.sampleMDX, profile);
    const secondCallTime = Date.now() - secondStartTime;

    // Results should be identical
    assert(
      JSON.stringify(result1) === JSON.stringify(result2),
      'Cached results should be identical to original'
    );

    // Second call should be faster (from cache)
    assert(
      secondCallTime < firstCallTime,
      `Cached call (${secondCallTime}ms) should be faster than first call (${firstCallTime}ms)`
    );

    this.testResults.push({ test: 'Cache functionality', status: 'PASS' });
  }

  /**
   * Test metadata generation
   */
  async testMetadataGeneration() {
    console.log('Testing Metadata Generation...');

    const profile = {
      experience_level: 'beginner',
      ros_familiarity: 'none',
      hardware_access: 'simulation_only',
      preferred_language: 'python'
    };

    const result = await this.adapter.adaptContent(this.sampleMDX, profile);

    // Check required metadata fields
    const requiredFields = [
      'difficulty',
      'reading_time_minutes',
      'adaptations_applied',
      'adapted_for',
      'adaptation_timestamp'
    ];

    for (const field of requiredFields) {
      assert(
        result.metadata.hasOwnProperty(field),
        `Missing required metadata field: ${field}`
      );
    }

    // Check adapted_for structure
    assert.deepEqual(
      result.metadata.adapted_for,
      profile,
      'adapted_for should match user profile'
    );

    // Check reading time is reasonable
    assert(
      result.metadata.reading_time_minutes > 0,
      'Reading time should be positive'
    );

    this.testResults.push({ test: 'Metadata generation', status: 'PASS' });
  }

  /**
   * Create sample MDX content for testing
   */
  createSampleMDX() {
    return `---
title: "Introduction to Robot Kinematics"
description: "Understanding robot kinematics fundamentals"
---

# Introduction to Robot Kinematics

Robot kinematics is the study of motion without considering forces. It's fundamental to understanding how robots move.

## Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector given joint configurations.

\`\`\`python
import numpy as np

def forward_kinematics(joint_angles, link_lengths):
    """Calculate end-effector position from joint angles"""
    x = sum(link_lengths[i] * np.cos(np.sum(joint_angles[:i+1])) for i in range(len(joint_angles)))
    y = sum(link_lengths[i] * np.sin(np.sum(joint_angles[:i+1])) for i in range(len(joint_angles)))
    return np.array([x, y])

# Example usage
joints = np.array([0.5, 0.3, 0.2])
links = [1.0, 0.8, 0.6]
position = forward_kinematics(joints, links)
print(f"End-effector position: {position}")
\`\`\`

## Inverse Kinematics

Inverse kinematics is the reverse problem - finding joint angles to reach a desired position.

## PID Control

PID controllers are widely used in robotics for precise control:

\`\`\`python
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
\`\`\`

## ROS Integration

This controller can be integrated with ROS for real robot control:

\`\`\`python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.pid = PIDController(1.0, 0.1, 0.05)
        # Hardware interface initialization
        self.hardware_interface = '/dev/ttyUSB0'
\`\`\`

## Hardware Deployment

When deploying to real hardware, ensure proper calibration and safety measures are in place.`;
  }

  /**
   * Create test user profiles
   */
  createTestProfiles() {
    return {
      beginner: {
        experience_level: 'beginner',
        ros_familiarity: 'none',
        hardware_access: 'simulation_only',
        preferred_language: 'python'
      },
      intermediate: {
        experience_level: 'intermediate',
        ros_familiarity: 'basic',
        hardware_access: 'partial_lab',
        preferred_language: 'python'
      },
      advanced: {
        experience_level: 'advanced',
        ros_familiarity: 'proficient',
        hardware_access: 'full_lab',
        preferred_language: 'cpp'
      }
    };
  }

  /**
   * Print test results summary
   */
  printResults() {
    console.log('\n📊 Test Results Summary:');
    console.log('========================');

    const passed = this.testResults.filter(r => r.status === 'PASS').length;
    const total = this.testResults.length;

    this.testResults.forEach(result => {
      console.log(`${result.status === 'PASS' ? '✅' : '❌'} ${result.test}`);
    });

    console.log(`\nTotal: ${passed}/${total} tests passed`);

    if (passed === total) {
      console.log('\n🎉 All tests passed! Content Adapter is working correctly.');
    } else {
      console.log(`\n⚠️  ${total - passed} test(s) failed. Please review the implementation.`);
    }
  }
}

/**
 * Generate sample adapted content for demonstration
 */
async function generateSampleAdaptations() {
  const adapter = new ContentAdapter();
  const sampleMDX = readFileSync('sample_chapter.mdx', 'utf-8');

  const profiles = [
    {
      name: 'Beginner Student',
      profile: {
        experience_level: 'beginner',
        ros_familiarity: 'none',
        hardware_access: 'simulation_only',
        preferred_language: 'python'
      }
    },
    {
      name: 'Advanced Engineer',
      profile: {
        experience_level: 'advanced',
        ros_familiarity: 'proficient',
        hardware_access: 'full_lab',
        preferred_language: 'cpp'
      }
    }
  ];

  for (const { name, profile } of profiles) {
    console.log(`\n📝 Generating adaptation for: ${name}`);
    console.log('Profile:', JSON.stringify(profile, null, 2));

    const result = await adapter.adaptContent(sampleMDX, profile);

    // Write adapted content to file
    const filename = `adapted_${name.toLowerCase().replace(/\s+/g, '_')}.mdx`;
    writeFileSync(filename, result.adapted_content);

    console.log(`✅ Saved adaptation to: ${filename}`);
    console.log(`Metadata:`, JSON.stringify(result.metadata, null, 2));
  }
}

// Run tests if called directly
if (import.meta.url === `file://${process.argv[1]}`) {
  const args = process.argv.slice(2);

  if (args.includes('--generate-samples')) {
    await generateSampleAdaptations();
  } else {
    const tests = new ContentAdapterTests();
    await tests.runAllTests();
  }
}

export { ContentAdapterTests };