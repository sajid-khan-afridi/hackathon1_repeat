---
name: mdx-writer
description: Write high-quality MDX files for Physical AI & Humanoid Robotics textbook chapters
version: 1.0.0
author: Claude AI Assistant
parameters:
  - name: chapter_title
    type: string
    required: true
    description: Title of the chapter
  - name: module_number
    type: integer
    required: true
    description: Module number (1-4)
    validation:
      min: 1
      max: 4
  - name: learning_objectives
    type: array
    required: true
    description: Array of learning objectives for the chapter
  - name: content_outline
    type: string
    required: true
    description: Structured outline of the chapter content
  - name: code_examples
    type: boolean
    required: false
    default: true
    description: Whether to include code examples
  - name: chapter_id
    type: string
    required: false
    description: Unique chapter identifier (auto-generated if not provided)
  - name: sidebar_position
    type: integer
    required: false
    description: Position in sidebar navigation
---

import { z } from 'zod';

// Input validation schema
const MDXSchema = z.object({
  chapter_title: z.string().min(1, "Chapter title is required"),
  module_number: z.number().int().min(1).max(4),
  learning_objectives: z.array(z.string()).min(1, "At least one learning objective is required"),
  content_outline: z.string().min(1, "Content outline is required"),
  code_examples: z.boolean().default(true),
  chapter_id: z.string().optional(),
  sidebar_position: z.number().int().positive().optional()
});

// MDX template components
const MDX_COMPONENTS = {
  admonitions: {
    tip: ':::tip[Tip]\n{{content}}\n:::',
    warning: ':::warning[Warning]\n{{content}}\n:::',
    note: ':::note[Note]\n{{content}}\n:::',
    important: ':::important[Important]\n{{content}}\n:::'
  },
  codeBlock: (language: string, code: string) => {
    return `\`\`\`${language}\n${code}\n\`\`\``;
  },
  personalizationButton: (chapterId: string) => {
    return `<PersonalizeButton chapterId="${chapterId}" />`;
  }
};

// Generate chapter ID from title if not provided
function generateChapterId(title: string): string {
  return title
    .toLowerCase()
    .replace(/[^a-z0-9\s-]/g, '')
    .replace(/\s+/g, '-')
    .substring(0, 50);
}

// Calculate sidebar position based on module number
function calculateSidebarPosition(moduleNumber: number, customPosition?: number): number {
  if (customPosition) return customPosition;
  return (moduleNumber - 1) * 10 + 1; // Module 1: 1-10, Module 2: 11-20, etc.
}

// Format learning objectives as checklist
function formatLearningObjectives(objectives: string[]): string {
  return objectives.map(obj => `- [ ] ${obj}`).join('\n');
}

// Generate MDX frontmatter
function generateFrontmatter(params: z.infer<typeof MDXSchema>): string {
  const chapterId = params.chapter_id || generateChapterId(params.chapter_title);
  const sidebarPosition = calculateSidebarPosition(params.module_number, params.sidebar_position);

  return `---
sidebar_position: ${sidebarPosition}
title: ${params.chapter_title}
description: ${params.chapter_title} - Module ${params.module_number}
chapter_id: ${chapterId}
module: ${params.module_number}
---`;
}

// Process content outline and add structured formatting
function processContent(outline: string, includeCodeExamples: boolean): string {
  // Split outline into sections
  const sections = outline.split(/\n(?=##\s)/);

  return sections.map(section => {
    // Add admonitions for important concepts
    if (section.includes('important') || section.includes('critical')) {
      section = section.replace(
        /(important|critical)/gi,
        MDX_COMPONENTS.admonitions.important.replace('{{content}}', '$1')
      );
    }

    // Add tips for best practices
    if (section.includes('best practice') || section.includes('tip')) {
      section = section.replace(
        /(best practice|tip)/gi,
        MDX_COMPONENTS.admonitions.tip.replace('{{content}}', '$1')
      );
    }

    // Add warnings for common pitfalls
    if (section.includes('warning') || section.includes('caution')) {
      section = section.replace(
        /(warning|caution)/gi,
        MDX_COMPONENTS.admonitions.warning.replace('{{content}}', '$1')
      );
    }

    return section;
  }).join('\n\n');
}

// Generate exercise section
function generateExercises(moduleNumber: number): string {
  const exerciseTemplates = {
    1: [
      "Implement a basic kinematic equation for a robotic joint",
      "Set up a simple ROS 2 node for position control",
      "Analyze the degrees of freedom in a given manipulator"
    ],
    2: [
      "Design a state estimator for a robotic system",
      "Implement a PID controller for joint movement",
      "Create a simulation scenario testing dynamic constraints"
    ],
    3: [
      "Develop a perception pipeline for object detection",
      "Implement a path planning algorithm using A*",
      "Design a decision-making system for task selection"
    ],
    4: [
      "Integrate multiple subsystems in a humanoid robot",
      "Design and test a safety-critical control system",
      "Implement human-robot interaction protocols"
    ]
  };

  const exercises = exerciseTemplates[moduleNumber as keyof typeof exerciseTemplates] || exerciseTemplates[1];

  return `## Exercises

${exercises.map((ex, i) => `### Exercise ${i + 1}: ${ex}

**Requirements:**
- Provide a clear implementation plan
- Include relevant code snippets
- Document expected outcomes and edge cases
- Test your solution with sample inputs

**Deliverables:**
1. Code implementation
2. Documentation explaining your approach
3. Test results and performance analysis`).join('\n\n')}`;
}

// Main MDX generation function
export async function execute(params: unknown): Promise<string> {
  // Validate inputs
  const validatedParams = MDXSchema.parse(params);
  const chapterId = validatedParams.chapter_id || generateChapterId(validatedParams.chapter_title);

  // Generate MDX content
  const frontmatter = generateFrontmatter(validatedParams);
  const objectivesList = formatLearningObjectives(validatedParams.learning_objectives);
  const processedContent = processContent(validatedParams.content_outline, validatedParams.code_examples);
  const exercises = generateExercises(validatedParams.module_number);

  // Build complete MDX
  const mdxContent = `${frontmatter}

# ${validatedParams.chapter_title}

${MDX_COMPONENTS.personalizationButton(chapterId)}

## Learning Objectives

${objectivesList}

## Introduction

This chapter explores fundamental concepts and practical applications in ${validatedParams.chapter_title.toLowerCase()}.
You will learn both theoretical foundations and hands-on implementation techniques that are essential
for working with physical AI and humanoid robotics systems.

${processedContent}

## Summary

In this chapter, we covered:
${validatedParams.learning_objectives.map(obj => `- ${obj}`).join('\n')}

These concepts form the foundation for advanced topics in physical AI and humanoid robotics.
Make sure you understand these principles before moving on to more complex implementations.

${exercises}

---

**Next Steps:** After completing these exercises, proceed to the next chapter to build upon these foundational concepts.
`;

  return mdxContent;
}