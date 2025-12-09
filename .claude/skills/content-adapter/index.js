#!/usr/bin/env node

import { readFileSync } from 'fs';
import { remark } from 'remark';
import remarkMdx from 'remark-mdx';
import remarkFrontmatter from 'remark-frontmatter';
import remarkParse from 'remark-parse';
import remarkStringify from 'remark-stringify';
import { matter } from 'vfile-matter';
import NodeCache from 'node-cache';
import { toString } from 'mdast-util-to-string';

/**
 * Content Adapter Skill
 * Dynamically adapts MDX content based on user profile
 */
class ContentAdapter {
  constructor(options = {}) {
    this.cache = new NodeCache({
      stdTTL: 3600, // 1 hour cache
      checkperiod: 600 // Check expired keys every 10 minutes
    });

    this.parser = remark()
      .use(remarkParse)
      .use(remarkMdx)
      .use(remarkFrontmatter);

    this.stringify = remark()
      .use(remarkStringify)
      .use(remarkMdx);
  }

  /**
   * Adapt MDX content based on user profile
   */
  async adaptContent(chapterContent, userProfile) {
    const cacheKey = this.generateCacheKey(chapterContent, userProfile);
    const cached = this.cache.get(cacheKey);

    if (cached) {
      return cached;
    }

    // Parse MDX content
    const mdast = this.parser.parse(chapterContent);

    // Extract frontmatter if present
    const vfile = { value: chapterContent };
    matter(vfile);
    const frontmatter = vfile.data.frontmatter || {};

    // Apply adaptations
    const adaptations = [];
    const adaptedMdast = await this.applyAdaptations(mdast, userProfile, adaptations);

    // Add metadata
    const metadata = this.generateMetadata(adaptedMdast, userProfile, adaptations);

    // Update frontmatter with adaptation metadata
    const adaptedContent = this.stringify.stringify({
      ...adaptedMdast,
      children: [
        {
          type: 'yaml',
          value: this.stringifyFrontmatter({
            ...frontmatter,
            ...metadata
          })
        },
        ...adaptedMdast.children.filter(child => child.type !== 'yaml')
      ]
    });

    const result = {
      adapted_content: String(adaptedContent),
      metadata
    };

    // Cache the result
    this.cache.set(cacheKey, result);

    return result;
  }

  /**
   * Apply all adaptation rules to the content
   */
  async applyAdaptations(mdast, profile, adaptations) {
    let adaptedMdast = { ...mdast };

    // Apply experience level adaptations
    adaptedMdast = await this.applyExperienceLevelAdaptation(adaptedMdast, profile.experience_level, adaptations);

    // Apply ROS familiarity adaptations
    adaptedMdast = await this.applyROSAdaptation(adaptedMdast, profile.ros_familiarity, adaptations);

    // Apply hardware access adaptations
    adaptedMdast = await this.applyHardwareAdaptation(adaptedMdast, profile.hardware_access, adaptations);

    // Apply language preferences
    adaptedMdast = await this.applyLanguageAdaptation(adaptedMdast, profile.preferred_language, adaptations);

    return adaptedMdast;
  }

  /**
   * Apply experience level adaptations
   */
  async applyExperienceLevelAdaptation(mdast, experienceLevel, adaptations) {
    if (!experienceLevel) return mdast;

    switch (experienceLevel.toLowerCase()) {
      case 'beginner':
        adaptations.push('beginner-simplification');
        return this.simplifyForBeginners(mdast);

      case 'intermediate':
        adaptations.push('intermediate-optimizations');
        return this.addIntermediateOptimizations(mdast);

      case 'advanced':
        adaptations.push('advanced-topics');
        return this.addAdvancedTopics(mdast);

      default:
        return mdast;
    }
  }

  /**
   * Apply ROS familiarity adaptations
   */
  async applyROSAdaptation(mdast, rosFamiliarity, adaptations) {
    if (!rosFamiliarity) return mdast;

    switch (rosFamiliarity.toLowerCase()) {
      case 'none':
        adaptations.push('ros-basics');
        return this.addROSBasicsSidebar(mdast);

      case 'basic':
        adaptations.push('ros-patterns');
        return this.addBasicROSPatterns(mdast);

      case 'proficient':
        adaptations.push('advanced-ros');
        return this.addAdvancedROSPatterns(mdast);

      default:
        return mdast;
    }
  }

  /**
   * Apply hardware access adaptations
   */
  async applyHardwareAdaptation(mdast, hardwareAccess, adaptations) {
    if (!hardwareAccess) return mdast;

    switch (hardwareAccess.toLowerCase()) {
      case 'simulation_only':
        adaptations.push('simulation-focus');
        return this.emphasizeSimulation(mdast);

      case 'partial_lab':
        adaptations.push('hybrid-examples');
        return this.addHybridExamples(mdast);

      case 'full_lab':
        adaptations.push('hardware-deployment');
        return this.addHardwareDeployment(mdast);

      default:
        return mdast;
    }
  }

  /**
   * Apply language preference adaptations
   */
  async applyLanguageAdaptation(mdast, language, adaptations) {
    if (!language || language.toLowerCase() === 'python') return mdast;

    adaptations.push(`language-${language}`);
    return this.adaptCodeExamples(mdast, language);
  }

  /**
   * Simplify content for beginners
   */
  simplifyForBeginners(mdast) {
    const adapted = { ...mdast };
    const insertions = [];

    adapted.children = adapted.children.map(node => {
      // Add prerequisite explanations for complex topics
      if (node.type === 'heading' && node.depth === 2) {
        const headingText = toString(node);
        if (this.isComplexTopic(headingText)) {
          insertions.push({
            type: 'definition',
            position: node.position,
            content: this.generatePrerequisiteExplanation(headingText)
          });
        }
      }

      // Simplify inline code blocks
      if (node.type === 'paragraph') {
        node.children = node.children.map(child => {
          if (child.type === 'inlineCode') {
            const simplified = this.simplifyCodeTerm(child.value);
            if (simplified !== child.value) {
              return {
                type: 'text',
                value: `${child.value} (${simplified})`
              };
            }
          }
          return child;
        });
      }

      return node;
    });

    // Insert explanations
    insertions.forEach(insertion => {
      const index = adapted.children.findIndex(n =>
        n.position && n.position.start.line === insertion.position.start.line
      );
      if (index > 0) {
        adapted.children.splice(index, 0, {
          type: 'blockquote',
          children: [
            {
              type: 'paragraph',
              children: [
                {
                  type: 'text',
                  value: `📚 **Prerequisite**: ${insertion.content}`
                }
              ]
            }
          ]
        });
      }
    });

    return adapted;
  }

  /**
   * Add optimization tips for intermediate users
   */
  addIntermediateOptimizations(mdast) {
    const adapted = { ...mdast };

    adapted.children = adapted.children.map(node => {
      // Add optimization tips after code blocks
      if (node.type === 'code') {
        const optimizationTip = this.generateOptimizationTip(node.value);
        if (optimizationTip) {
          return [
            node,
            {
              type: 'callout',
              children: [
                {
                  type: 'paragraph',
                  children: [
                    {
                      type: 'text',
                      value: `💡 **Optimization Tip**: ${optimizationTip}`
                    }
                  ]
                }
              ]
            }
          ];
        }
      }
      return node;
    }).flat();

    return adapted;
  }

  /**
   * Add advanced topics and research references
   */
  addAdvancedTopics(mdast) {
    const adapted = { ...mdast };
    const advancedSection = {
      type: 'section',
      children: [
        {
          type: 'heading',
          depth: 3,
          children: [
            {
              type: 'text',
              value: 'Advanced Topics & Further Reading'
            }
          ]
        },
        {
          type: 'paragraph',
          children: [
            {
              type: 'text',
              value: 'For deeper understanding, consider exploring these research papers and advanced concepts:'
            }
          ]
        },
        {
          type: 'list',
          ordered: true,
          children: this.generateAdvancedReadings()
        }
      ]
    };

    adapted.children.push(advancedSection);
    return adapted;
  }

  /**
   * Add ROS basics sidebar for beginners
   */
  addROSBasicsSidebar(mdast) {
    const adapted = { ...mdast };

    const rosbasicsSidebar = {
      type: 'aside',
      children: [
        {
          type: 'heading',
          depth: 4,
          children: [
            {
              type: 'text',
              value: '🤖 ROS Basics'
            }
          ]
        },
        {
          type: 'paragraph',
          children: [
            {
              type: 'text',
              value: 'New to ROS? Here are the essential concepts you need to know:'
            }
          ]
        },
        {
          type: 'list',
          unordered: true,
          children: [
            {
              type: 'listItem',
              children: [
                {
                  type: 'paragraph',
                  children: [
                    {
                      type: 'strong',
                      children: [{ type: 'text', value: 'Nodes' }]
                    },
                    {
                      type: 'text',
                      value: ': Processes that perform computation'
                    }
                  ]
                }
              ]
            },
            {
              type: 'listItem',
              children: [
                {
                  type: 'paragraph',
                  children: [
                    {
                      type: 'strong',
                      children: [{ type: 'text', value: 'Topics' }]
                    },
                    {
                      type: 'text',
                      value: ': Named buses for message exchange'
                    }
                  ]
                }
              ]
            },
            {
              type: 'listItem',
              children: [
                {
                  type: 'paragraph',
                  children: [
                    {
                      type: 'strong',
                      children: [{ type: 'text', value: 'Messages' }]
                    },
                    {
                      type: 'text',
                      value: ': Data structures for communication'
                    }
                  ]
                }
              ]
            }
          ]
        }
      ]
    };

    adapted.children.splice(1, 0, rosbasicsSidebar);
    return adapted;
  }

  /**
   * Add advanced ROS patterns
   */
  addAdvancedROSPatterns(mdast) {
    const adapted = { ...mdast };

    // Add advanced ROS patterns after relevant sections
    adapted.children = adapted.children.map(node => {
      if (node.type === 'heading' && toString(node).toLowerCase().includes('ros')) {
        return [
          node,
          {
            type: 'advancedPattern',
            children: [
              {
                type: 'heading',
                depth: 4,
                children: [
                  {
                    type: 'text',
                    value: 'Advanced ROS Patterns'
                  }
                ]
              },
              {
                type: 'paragraph',
                children: [
                  {
                    type: 'text',
                    value: 'Consider these advanced ROS patterns for production systems:'
                  }
                ]
              },
              {
                type: 'code',
                lang: 'python',
                value: this.generateAdvancedROSPattern()
              }
            ]
          }
        ];
      }
      return node;
    }).flat();

    return adapted;
  }

  /**
   * Emphasize simulation for users without hardware
   */
  emphasizeSimulation(mdast) {
    const adapted = { ...mdast };

    adapted.children = adapted.children.map(node => {
      // Convert hardware-specific examples to simulation equivalents
      if (node.type === 'code' && node.value.includes('hardware')) {
        const simulationVersion = this.convertToSimulation(node.value);
        if (simulationVersion !== node.value) {
          node.value = simulationVersion;
          node.meta = 'simulation-focused';
        }
      }

      // Add simulation notes
      if (node.type === 'paragraph' && toString(node).toLowerCase().includes('hardware')) {
        const children = [...node.children];
        children.push({
          type: 'emphasis',
          children: [
            {
              type: 'text',
              value: ' (Simulation equivalent available)'
            }
          ]
        });
        return { ...node, children };
      }

      return node;
    });

    return adapted;
  }

  /**
   * Add hardware deployment sections
   */
  addHardwareDeployment(mdast) {
    const adapted = { ...mdast };

    const hardwareSection = {
      type: 'section',
      children: [
        {
          type: 'heading',
          depth: 3,
          children: [
            {
              type: 'text',
              value: 'Hardware Deployment'
            }
          ]
        },
        {
          type: 'paragraph',
          children: [
            {
              type: 'text',
              value: 'Deploy this system on physical hardware with these considerations:'
            }
          ]
        },
        {
          type: 'list',
          unordered: true,
          children: this.generateHardwareChecklist()
        }
      ]
    };

    adapted.children.push(hardwareSection);
    return adapted;
  }

  /**
   * Adapt code examples to preferred language
   */
  adaptCodeExamples(mdast, language) {
    const adapted = { ...mdast };

    adapted.children = adapted.children.map(node => {
      if (node.type === 'code' && node.lang === 'python' && language !== 'python') {
        const convertedCode = this.convertCode(node.value, 'python', language);
        if (convertedCode) {
          return {
            ...node,
            lang: language,
            value: convertedCode
          };
        }
      }
      return node;
    });

    return adapted;
  }

  /**
   * Generate metadata for the adapted content
   */
  generateMetadata(mdast, profile, adaptations) {
    const textContent = toString(mdast);
    const wordsPerMinute = 200;
    const wordCount = textContent.split(/\s+/).length;
    const baseReadingTime = Math.ceil(wordCount / wordsPerMinute);

    // Adjust reading time based on adaptations
    let readingTimeMultiplier = 1.0;
    if (adaptations.includes('beginner-simplification')) readingTimeMultiplier += 0.3;
    if (adaptations.includes('advanced-topics')) readingTimeMultiplier += 0.4;
    if (adaptations.includes('ros-basics')) readingTimeMultiplier += 0.2;
    if (adaptations.includes('hardware-deployment')) readingTimeMultiplier += 0.2;

    return {
      difficulty: this.getDifficultyBadge(profile),
      reading_time_minutes: Math.ceil(baseReadingTime * readingTimeMultiplier),
      adaptations_applied: adaptations,
      adapted_for: {
        experience_level: profile.experience_level,
        ros_familiarity: profile.ros_familiarity,
        hardware_access: profile.hardware_access,
        language: profile.preferred_language
      },
      adaptation_timestamp: new Date().toISOString()
    };
  }

  /**
   * Get difficulty badge based on user profile
   */
  getDifficultyBadge(profile) {
    const { experience_level } = profile;

    switch (experience_level?.toLowerCase()) {
      case 'beginner':
        return '🟢 Beginner-Friendly';
      case 'intermediate':
        return '🟡 Intermediate';
      case 'advanced':
        return '🔴 Advanced';
      default:
        return '⚪ Standard';
    }
  }

  /**
   * Generate cache key for content and profile
   */
  generateCacheKey(content, profile) {
    const contentHash = this.simpleHash(content);
    const profileHash = this.simpleHash(JSON.stringify(profile));
    return `${contentHash}-${profileHash}`;
  }

  /**
   * Simple hash function for cache keys
   */
  simpleHash(str) {
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
      const char = str.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash;
    }
    return Math.abs(hash).toString(36);
  }

  /**
   * Stringify frontmatter
   */
  stringifyFrontmatter(data) {
    return Object.entries(data)
      .map(([key, value]) => {
        if (typeof value === 'string') {
          return `${key}: ${JSON.stringify(value)}`;
        } else if (Array.isArray(value)) {
          return `${key}: [${value.map(v => JSON.stringify(v)).join(', ')}]`;
        } else if (typeof value === 'object') {
          return `${key}: ${JSON.stringify(value)}`;
        }
        return `${key}: ${value}`;
      })
      .join('\n');
  }

  // Helper methods for content generation
  isComplexTopic(heading) {
    const complexTerms = ['kinematics', 'dynamics', 'optimization', 'kalman', 'control theory'];
    return complexTerms.some(term => heading.toLowerCase().includes(term));
  }

  generatePrerequisiteExplanation(topic) {
    const explanations = {
      'kinematics': 'Kinematics studies motion without considering forces. It focuses on positions, velocities, and accelerations.',
      'dynamics': 'Dynamics considers the forces that cause motion. It includes concepts like torque, inertia, and momentum.',
      'optimization': 'Optimization finds the best solution from a set of alternatives by minimizing or maximizing an objective function.',
      'kalman': 'Kalman filtering is an algorithm that uses noisy measurements to produce estimates of unknown variables.'
    };

    const lowerTopic = topic.toLowerCase();
    for (const [key, explanation] of Object.entries(explanations)) {
      if (lowerTopic.includes(key)) return explanation;
    }
    return 'This topic builds on fundamental concepts in robotics and mathematics.';
  }

  simplifyCodeTerm(term) {
    const simplifications = {
      'PID': 'Proportional-Integral-Derivative controller',
      'TF': 'Transformation frame',
      'ROS': 'Robot Operating System',
      'SLAM': 'Simultaneous Localization and Mapping'
    };

    return simplifications[term] || term;
  }

  generateOptimizationTip(code) {
    if (code.includes('for')) return 'Consider using numpy vectorization instead of loops when possible.';
    if (code.includes('list')) return 'Use array operations or generators for memory efficiency with large datasets.';
    if (code.includes('class')) return 'Consider using dataclasses for cleaner data structures with automatic __init__ methods.';
    return null;
  }

  generateAdvancedReadings() {
    return [
      {
        type: 'listItem',
        children: [
          {
            type: 'paragraph',
            children: [
              {
                type: 'text',
                value: '"Probabilistic Robotics" by Thrun, Burgard, and Fox - Essential reading for state estimation and mapping'
              }
            ]
          }
        ]
      },
      {
        type: 'listItem',
        children: [
          {
            type: 'paragraph',
            children: [
              {
                type: 'text',
                value: '"Modern Robotics: Mechanics, Planning, and Control" by Lynch and Park - Comprehensive treatment of robot kinematics and dynamics'
              }
            ]
          }
        ]
      }
    ];
  }

  generateAdvancedROSPattern() {
    return `# Advanced ROS Pattern: Multi-Process Node Architecture
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

class AdvancedRobotNode(Node):
    def __init__(self):
        super().__init__('advanced_robot_node')
        self.executor = MultiThreadedExecutor()
        self.setup_component_processes()

    async def setup_component_processes(self):
        # Create separate processes for each component
        self.navigation_task = asyncio.create_task(self.navigation_loop())
        self.perception_task = asyncio.create_task(self.perception_loop())
        self.control_task = asyncio.create_task(self.control_loop())`;
  }

  convertToSimulation(code) {
    return code.replace('/dev/ttyUSB0', 'localhost:5555')
              .replace('hardware_interface', 'gazebo_interface')
              .replace('real_robot', 'simulated_robot');
  }

  generateHardwareChecklist() {
    return [
      {
        type: 'listItem',
        children: [
          {
            type: 'paragraph',
            children: [
              {
                type: 'text',
                value: '✅ Calibrate sensors according to manufacturer specifications'
              }
            ]
          }
        ]
      },
      {
        type: 'listItem',
        children: [
          {
            type: 'paragraph',
            children: [
              {
                type: 'text',
                value: '⚡ Verify power supply can handle peak current requirements'
              }
            ]
          }
        ]
      },
      {
        type: 'listItem',
        children: [
          {
            type: 'paragraph',
            children: [
              {
                type: 'text',
                value: '🔧 Implement emergency stop mechanisms and safety interlocks'
              }
            ]
          }
        ]
      }
    ];
  }

  convertCode(code, fromLang, toLang) {
    // Simplified code conversion - in practice, use ast or specialized tools
    if (fromLang === 'python' && toLang === 'cpp') {
      return `// C++ equivalent of the Python code
#include <iostream>
#include <vector>
int main() {
    // Converted from Python
    std::cout << "Code adapted for C++" << std::endl;
    return 0;
}`;
    }
    return null;
  }
}

// CLI interface
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 2) {
    console.error('Usage: node index.js <input_file> <user_profile_json>');
    process.exit(1);
  }

  const inputFile = args[0];
  const userProfile = JSON.parse(args[1]);

  const adapter = new ContentAdapter();
  const chapterContent = readFileSync(inputFile, 'utf-8');

  const result = await adapter.adaptContent(chapterContent, userProfile);

  console.log(JSON.stringify(result, null, 2));
}

// Export for use as a module
export default ContentAdapter;

// Run CLI if called directly
if (import.meta.url === `file://${process.argv[1]}`) {
  main().catch(console.error);
}