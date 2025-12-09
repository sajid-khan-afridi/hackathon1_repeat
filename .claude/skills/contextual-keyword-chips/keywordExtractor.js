/**
 * Keyword Extraction Module
 * Uses NLP techniques to extract relevant keywords from AI responses
 */

import nlp from 'compromise';

export class KeywordExtractor {
  constructor(options = {}) {
    this.maxChips = options.maxChips || 5;
    this.minWordLength = options.minWordLength || 3;
    this.excludeWords = new Set([
      'the', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with',
      'by', 'from', 'up', 'about', 'into', 'through', 'during', 'before',
      'after', 'above', 'below', 'between', 'among', 'is', 'are', 'was', 'were',
      'be', 'been', 'being', 'have', 'has', 'had', 'do', 'does', 'did',
      'will', 'would', 'could', 'should', 'may', 'might', 'must', 'can',
      'this', 'that', 'these', 'those', 'i', 'you', 'he', 'she', 'it', 'we', 'they',
      'what', 'which', 'who', 'when', 'where', 'why', 'how', 'a', 'an',
      'AI', 'ai', 'artificial', 'intelligence', 'please', 'thank', 'thanks'
    ]);
    this.techKeywords = new Set([
      'React', 'Vue', 'Angular', 'NextJS', 'NodeJS', 'Python', 'JavaScript',
      'TypeScript', 'API', 'REST', 'GraphQL', 'Docker', 'Kubernetes',
      'AWS', 'Azure', 'GCP', 'MongoDB', 'PostgreSQL', 'MySQL', 'Redis',
      'Git', 'GitHub', 'GitLab', 'CI/CD', 'DevOps', 'Microservices',
      'Serverless', 'Lambda', 'Functions', 'Authentication', 'Authorization',
      'OAuth', 'JWT', 'JWTs', 'HTTP', 'HTTPS', 'SSL', 'TLS', 'Encryption',
      'Blockchain', 'Smart Contract', 'DeFi', 'NFT', 'Web3', 'Metaverse',
      'Machine Learning', 'ML', 'Deep Learning', 'Neural Network',
      'TensorFlow', 'PyTorch', 'Scikit-learn', 'Pandas', 'NumPy',
      'Data Science', 'Analytics', 'Visualization', 'Dashboard',
      'Agile', 'Scrum', 'Kanban', 'TDD', 'BDD', 'SOLID', 'DRY', 'KISS',
      'MVC', 'MVVM', 'MVP', 'Component', 'Hook', 'State', 'Props',
      'Redux', 'MobX', 'Context', 'Async', 'Await', 'Promise', 'Callback',
      'JSON', 'XML', 'YAML', 'Markdown', 'HTML', 'CSS', 'SASS', 'SCSS',
      'Webpack', 'Vite', 'Parcel', 'Rollup', 'Babel', 'ESLint', 'Prettier',
      'Testing', 'Jest', 'Mocha', 'Cypress', 'Playwright', 'Selenium'
    ]);
  }

  extractKeywords(text) {
    if (!text || typeof text !== 'string') {
      return [];
    }

    const doc = nlp(text);
    const keywords = new Map();

    // Extract entities
    const entities = doc.entities().json();
    entities.forEach(entity => {
      const text = entity.text;
      if (text.length >= this.minWordLength) {
        keywords.set(text.toLowerCase(), {
          text: text,
          type: 'entity',
          score: 2.0
        });
      }
    });

    // Extract nouns and noun phrases
    const nouns = doc.nouns().json();
    nouns.forEach(noun => {
      const text = noun.text;
      if (text && text.length >= this.minWordLength && !this.excludeWords.has(text.toLowerCase())) {
        const existing = keywords.get(text.toLowerCase());
        if (!existing || existing.score < 1.5) {
          keywords.set(text.toLowerCase(), {
            text: text,
            type: 'noun',
            score: 1.5
          });
        }
      }
    });

    // Extract adjectives that might be relevant
    const adjectives = doc.adjectives().json();
    adjectives.forEach(adj => {
      const text = adj.text;
      if (text && text.length >= this.minWordLength && !this.excludeWords.has(text.toLowerCase())) {
        keywords.set(text.toLowerCase(), {
          text: text,
          type: 'adjective',
          score: 1.0
        });
      }
    });

    // Extract tech terms and capitalized words
    const techMatches = text.match(/\b[A-Z][a-zA-Z]+\b/g) || [];
    techMatches.forEach(term => {
      if (term.length >= this.minWordLength) {
        const existing = keywords.get(term.toLowerCase());
        if (!existing || existing.score < 2.5) {
          keywords.set(term.toLowerCase(), {
            text: term,
            type: this.techKeywords.has(term) ? 'tech' : 'proper',
            score: this.techKeywords.has(term) ? 3.0 : 2.0
          });
        }
      }
    });

    // Extract patterns for actions/tasks
    const actionPatterns = [
      /\b(create|build|implement|develop|design|write|code|test|deploy|run|execute|setup|configure)\s+\w+\b/gi,
      /\b(how|what|why|when|where)\s+to\s+\w+/gi,
      /\b(optimization|performance|security|integration|authentication|authorization)\b/gi
    ];

    actionPatterns.forEach(pattern => {
      const matches = text.match(pattern) || [];
      matches.forEach(match => {
        keywords.set(match.toLowerCase(), {
          text: match,
          type: 'action',
          score: 2.5
        });
      });
    });

    // Convert to array and sort by score
    const sortedKeywords = Array.from(keywords.values())
      .sort((a, b) => b.score - a.score)
      .slice(0, this.maxChips);

    // Generate follow-up questions based on context
    const followUpQuestions = this.generateFollowUpQuestions(text, sortedKeywords);

    return [...sortedKeywords, ...followUpQuestions].slice(0, this.maxChips);
  }

  generateFollowUpQuestions(text, keywords) {
    const questions = [];
    const questionTemplates = [
      'What is {keyword}?',
      'How does {keyword} work?',
      'How to implement {keyword}?',
      '{keyword} best practices',
      'Examples of {keyword}',
      'Learn more about {keyword}'
    ];

    // Use top keywords to generate questions
    const topKeywords = keywords.slice(0, 2).map(k => k.text);

    topKeywords.forEach(keyword => {
      // Find most relevant template
      const doc = nlp(text);
      const hasHowTo = text.toLowerCase().includes('how to') || text.toLowerCase().includes('how can');
      const hasWhat = text.toLowerCase().includes('what is') || text.toLowerCase().includes('what are');
      const hasExamples = text.toLowerCase().includes('example') || text.toLowerCase().includes('demo');

      let template;
      if (hasHowTo) {
        template = 'More about {keyword}';
      } else if (hasWhat) {
        template = 'How to use {keyword}';
      } else if (hasExamples) {
        template = 'Advanced {keyword} techniques';
      } else {
        template = questionTemplates[Math.floor(Math.random() * questionTemplates.length)];
      }

      questions.push({
        text: template.replace('{keyword}', keyword),
        type: 'question',
        score: 1.8
      });
    });

    return questions;
  }

  categorizeChips(keywords) {
    return {
      entities: keywords.filter(k => k.type === 'entity'),
      concepts: keywords.filter(k => k.type === 'noun'),
      actions: keywords.filter(k => k.type === 'action'),
      questions: keywords.filter(k => k.type === 'question'),
      tech: keywords.filter(k => k.type === 'tech')
    };
  }
}