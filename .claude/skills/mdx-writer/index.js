#!/usr/bin/env node

import fs from 'fs';
import path from 'path';

/**
 * MDX Writer Skill
 * Creates high-quality MDX files for technical documentation
 */
class MdxWriter {
  constructor(options = {}) {
    this.projectRoot = options.projectRoot || process.cwd();
    this.docsDir = options.docsDir || path.join(this.projectRoot, 'docs');
  }

  /**
   * Generate an MDX chapter file
   */
  async createChapter(options = {}) {
    const {
      chapterTitle,
      moduleNumber,
      learningObjectives = [],
      contentOutline = '',
      codeExamples = { include: true, languages: ['python'], showOutput: true },
      exercises = { include: true, difficulty: 'mixed', count: 3 },
      chapterId,
      sidebarPosition
    } = options;

    if (!chapterTitle || !moduleNumber) {
      throw new Error('chapterTitle and moduleNumber are required');
    }

    // Generate chapter ID from title
    const generatedChapterId = chapterId || this.generateChapterId(chapterTitle);

    // Calculate sidebar position
    const generatedPosition = sidebarPosition || this.calculateSidebarPosition(moduleNumber);

    // Generate tags from title
    const tags = this.generateTags(chapterTitle, moduleNumber);

    // Build frontmatter
    const frontmatter = {
      title: chapterTitle,
      sidebar_position: generatedPosition,
      chapter_id: generatedChapterId,
      module: moduleNumber,
      tags
    };

    // Generate MDX content
    const mdxContent = this.buildMdxContent({
      frontmatter,
      learningObjectives,
      contentOutline,
      codeExamples,
      exercises
    });

    // Determine output path
    const moduleDir = path.join(this.docsDir, `module-${moduleNumber}`);
    if (!fs.existsSync(moduleDir)) {
      fs.mkdirSync(moduleDir, { recursive: true });
    }

    const filename = `${generatedChapterId}.mdx`;
    const outputPath = path.join(moduleDir, filename);

    // Write file
    fs.writeFileSync(outputPath, mdxContent, 'utf-8');

    // Calculate statistics
    const wordCount = mdxContent.split(/\s+/).length;
    const codeExamplesCount = (mdxContent.match(/```/g) || []).length / 2;
    const exercisesCount = (mdxContent.match(/##.*Exercise/gi) || []).length;

    return {
      success: true,
      mdx_file_path: outputPath,
      frontmatter,
      learning_objectives_count: learningObjectives.length,
      code_examples_count: codeExamplesCount,
      exercises_count: exercisesCount,
      word_count: wordCount
    };
  }

  /**
   * Build complete MDX content
   */
  buildMdxContent(options) {
    const { frontmatter, learningObjectives, contentOutline, codeExamples, exercises } = options;

    let content = [];

    // Frontmatter
    content.push('---');
    content.push(`title: "${frontmatter.title}"`);
    content.push(`sidebar_position: ${frontmatter.sidebar_position}`);
    content.push(`chapter_id: "${frontmatter.chapter_id}"`);
    content.push(`module: ${frontmatter.module}`);
    content.push(`tags: [${frontmatter.tags.map(t => `"${t}"`).join(', ')}]`);
    content.push('---');
    content.push('');

    // Import statements
    content.push('import Tabs from \'@theme/Tabs\';');
    content.push('import TabItem from \'@theme/TabItem\';');
    content.push('');

    // Title and learning objectives
    content.push(`# ${frontmatter.title}`);
    content.push('');

    if (learningObjectives.length > 0) {
      content.push('## Learning Objectives');
      content.push('');
      content.push('By the end of this chapter, you will be able to:');
      content.push('');
      learningObjectives.forEach(obj => {
        content.push(`- ${obj}`);
      });
      content.push('');
    }

    // Main content
    if (contentOutline) {
      content.push(contentOutline);
      content.push('');
    }

    // Code examples section
    if (codeExamples.include) {
      content.push('## Code Examples');
      content.push('');
      content.push(this.generateCodeExamplePlaceholder(codeExamples));
      content.push('');
    }

    // Exercises section
    if (exercises.include) {
      content.push('## Exercises');
      content.push('');
      content.push(this.generateExercisesPlaceholder(exercises));
      content.push('');
    }

    // Summary
    content.push('## Summary');
    content.push('');
    content.push('In this chapter, we covered:');
    content.push('');
    learningObjectives.slice(0, 3).forEach(obj => {
      content.push(`- ${obj}`);
    });
    content.push('');

    // Next steps
    content.push('## Next Steps');
    content.push('');
    content.push('Continue to the next chapter to learn more advanced concepts.');
    content.push('');

    return content.join('\n');
  }

  /**
   * Generate code example placeholder
   */
  generateCodeExamplePlaceholder(config) {
    const lines = [];
    const lang = config.languages[0] || 'python';

    lines.push(`<Tabs>`);
    config.languages.forEach(language => {
      lines.push(`<TabItem value="${language}" label="${language.toUpperCase()}">`);
      lines.push('');
      lines.push(`\`\`\`${language}`);
      lines.push(`# ${language.toUpperCase()} code example`);
      lines.push(`# TODO: Add ${language} implementation`);
      lines.push('```');

      if (config.showOutput) {
        lines.push('');
        lines.push('**Expected Output:**');
        lines.push('```');
        lines.push('# Output will appear here');
        lines.push('```');
      }

      lines.push('');
      lines.push('</TabItem>');
    });
    lines.push('</Tabs>');

    return lines.join('\n');
  }

  /**
   * Generate exercises placeholder
   */
  generateExercisesPlaceholder(config) {
    const lines = [];
    const difficulties = config.difficulty === 'mixed'
      ? ['beginner', 'intermediate', 'advanced']
      : [config.difficulty];

    for (let i = 1; i <= config.count; i++) {
      const difficulty = difficulties[(i - 1) % difficulties.length];
      const emoji = difficulty === 'beginner' ? '🟢' : difficulty === 'intermediate' ? '🟡' : '🔴';

      lines.push(`### Exercise ${i}: ${emoji} ${this.capitalizeFirst(difficulty)}`);
      lines.push('');
      lines.push('**Task:** [Describe the exercise task here]');
      lines.push('');
      lines.push('**Hints:**');
      lines.push('1. [First hint]');
      lines.push('2. [Second hint]');
      lines.push('');
      lines.push('<details>');
      lines.push('<summary>Solution</summary>');
      lines.push('');
      lines.push('```python');
      lines.push('# Solution code here');
      lines.push('```');
      lines.push('');
      lines.push('</details>');
      lines.push('');
    }

    return lines.join('\n');
  }

  /**
   * Generate chapter ID from title
   */
  generateChapterId(title) {
    return title
      .toLowerCase()
      .replace(/[^a-z0-9]+/g, '-')
      .replace(/^-|-$/g, '');
  }

  /**
   * Calculate sidebar position
   */
  calculateSidebarPosition(moduleNumber) {
    return moduleNumber * 10 + 1;
  }

  /**
   * Generate tags from title
   */
  generateTags(title, moduleNumber) {
    const keywords = title.toLowerCase().split(/\s+/)
      .filter(word => word.length > 3)
      .slice(0, 3);

    return [`module-${moduleNumber}`, ...keywords];
  }

  /**
   * Capitalize first letter
   */
  capitalizeFirst(str) {
    return str.charAt(0).toUpperCase() + str.slice(1);
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: mdx-writer create --title "Chapter Title" --module 1');
    console.error('');
    console.error('Options:');
    console.error('  --title      Chapter title (required)');
    console.error('  --module     Module number 1-4 (required)');
    console.error('  --objectives Learning objectives (comma-separated)');
    console.error('  --languages  Code languages (comma-separated, default: python)');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const writer = new MdxWriter({ projectRoot: process.cwd() });

  try {
    let result;
    switch (operation) {
      case 'create':
        result = await writer.createChapter({
          chapterTitle: options.title,
          moduleNumber: parseInt(options.module),
          learningObjectives: options.objectives ? options.objectives.split(',') : [],
          codeExamples: {
            include: true,
            languages: options.languages ? options.languages.split(',') : ['python'],
            showOutput: true
          },
          exercises: {
            include: true,
            difficulty: options.difficulty || 'mixed',
            count: parseInt(options.exercises || '3')
          }
        });
        break;

      default:
        console.error('Unknown operation:', operation);
        process.exit(1);
    }

    console.log(JSON.stringify(result, null, 2));
  } catch (error) {
    console.error('Error:', error.message);
    process.exit(1);
  }
}

function parseCliArgs(args) {
  const options = {};
  for (let i = 0; i < args.length; i++) {
    if (args[i].startsWith('--')) {
      const key = args[i].slice(2);
      const value = args[i + 1] && !args[i + 1].startsWith('--') ? args[i + 1] : 'true';
      options[key] = value;
      if (value !== 'true') i++;
    }
  }
  return options;
}

export default MdxWriter;
export { MdxWriter };

const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('mdx-writer/index.js');
if (isMainModule) {
  main().catch(console.error);
}
