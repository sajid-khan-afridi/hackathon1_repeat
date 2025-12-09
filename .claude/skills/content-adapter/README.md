# Content Adapter Skill

A skill that dynamically adapts MDX chapter content based on user profiles to provide personalized learning experiences.

## Features

- **Experience Level Adaptation**: Adjusts content complexity for beginners, intermediate, and advanced users
- **ROS Familiarity Adaptation**: Adds ROS basics for newcomers or advanced patterns for experts
- **Hardware Access Adaptation**: Emphasizes simulation or includes hardware deployment based on available resources
- **Language Preference**: Converts code examples to user's preferred programming language
- **Smart Caching**: Efficient caching of adapted content to improve performance
- **Metadata Generation**: Provides difficulty badges, reading time estimates, and adaptation tracking

## Usage

### As a Module

```javascript
import ContentAdapter from './index.js';

const adapter = new ContentAdapter();

const userProfile = {
  experience_level: 'beginner',
  ros_familiarity: 'none',
  hardware_access: 'simulation_only',
  preferred_language: 'python'
};

const result = await adapter.adaptContent(chapterMDX, userProfile);
console.log(result.adapted_content);
console.log(result.metadata);
```

### From Command Line

```bash
node index.js chapter.mdx '{"experience_level":"beginner","ros_familiarity":"none"}'
```

## User Profile Schema

```javascript
{
  experience_level: "beginner" | "intermediate" | "advanced",
  ros_familiarity: "none" | "basic" | "proficient",
  hardware_access: "simulation_only" | "partial_lab" | "full_lab",
  preferred_language: string (default: "python")
}
```

## Adaptation Rules

### Experience Levels

1. **Beginner**:
   - Adds prerequisite explanations
   - Simplifies technical jargon with inline definitions
   - Provides additional context for complex topics
   - Difficulty badge: 🟢 Beginner-Friendly

2. **Intermediate**:
   - Standard content with optimization tips
   - Code performance suggestions
   - Best practices highlights
   - Difficulty badge: 🟡 Intermediate

3. **Advanced**:
   - Adds advanced topics and research references
   - Links to academic papers
   - Advanced implementation patterns
   - Difficulty badge: 🔴 Advanced

### ROS Familiarity

1. **No ROS**:
   - Includes ROS basics sidebar
   - Explains core concepts (nodes, topics, messages)
   - Provides beginner-friendly introductions

2. **Basic ROS**:
   - Adds common ROS patterns
   - Includes basic best practices
   - Shows standard node structures

3. **Proficient ROS**:
   - Skips basic explanations
   - Adds advanced ROS patterns
   - Includes multi-threading and architecture patterns

### Hardware Access

1. **Simulation Only**:
   - Focuses on Gazebo/Isaac Sim examples
   - Converts hardware-specific code to simulation equivalents
   - Adds simulation-focused notes

2. **Partial Lab**:
   - Provides hybrid examples
   - Shows both simulation and hardware approaches
   - Includes transition guidance

3. **Full Lab**:
   - Includes hardware deployment sections
   - Adds safety and calibration checklists
   - Provides real-world implementation details

## Output Schema

```javascript
{
  adapted_content: string, // Adapted MDX content
  metadata: {
    difficulty: string, // Difficulty badge
    reading_time_minutes: number, // Adjusted reading time
    adaptations_applied: string[], // List of applied adaptations
    adapted_for: {
      experience_level: string,
      ros_familiarity: string,
      hardware_access: string,
      language: string
    },
    adaptation_timestamp: string // ISO timestamp
  }
}
```

## Testing

Run the test suite:

```bash
node test.js
```

Generate sample adaptations:

```bash
node test.js --generate-samples
```

## Dependencies

- `gray-matter`: Frontmatter parsing
- `remark`: MDX processing pipeline
- `remark-mdx`: MDX support
- `node-cache`: Content caching
- `unified`: Processor utilities

## Cache Configuration

- Cache TTL: 1 hour
- Check period: 10 minutes
- Cache key: Hash of content + profile combination

## Architecture

The Content Adapter follows a pipeline approach:

1. **Parse**: MDX content is parsed into an AST
2. **Extract**: Frontmatter and content structure is extracted
3. **Adapt**: Profile-specific transformations are applied
4. **Generate**: Metadata is calculated
5. **Serialize**: AST is converted back to MDX
6. **Cache**: Results are cached for future use

## Performance Considerations

- Content is cached to avoid repeated processing
- Adaptations are applied incrementally
- Heavy computations are avoided where possible
- Hash-based cache keys ensure efficient lookup

## Future Enhancements

- Support for more programming languages
- Machine learning-based content difficulty assessment
- Dynamic quiz generation based on adapted content
- Integration with learning management systems
- A/B testing support for content variations