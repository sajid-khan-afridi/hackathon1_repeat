#!/usr/bin/env node

import fs from 'fs';
import path from 'path';
import { Pool } from 'pg';

/**
 * Database Migrations Skill
 * Manage PostgreSQL database schema migrations for the robotics textbook platform
 */
class DbMigrations {
  constructor(options = {}) {
    this.projectRoot = options.projectRoot || process.cwd();
    this.migrationsDir = options.migrationsDir || path.join(this.projectRoot, 'migrations');
    this.pool = null;

    // Schema definitions for all tables
    this.schemas = {
      users: {
        description: 'User authentication and account information',
        sql: `
          CREATE TABLE IF NOT EXISTS users (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            email VARCHAR(255) UNIQUE NOT NULL,
            password_hash VARCHAR(255),
            name VARCHAR(255),
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
          );
          CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
        `
      },
      user_profiles: {
        description: 'User profile data for personalization',
        sql: `
          CREATE TABLE IF NOT EXISTS user_profiles (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            user_id UUID REFERENCES users(id) ON DELETE CASCADE,
            experience_level VARCHAR(20) CHECK (experience_level IN ('beginner', 'intermediate', 'advanced')),
            ros_familiarity VARCHAR(20) CHECK (ros_familiarity IN ('none', 'basic', 'proficient')),
            hardware_access VARCHAR(20) CHECK (hardware_access IN ('simulation_only', 'partial_lab', 'full_lab')),
            learning_goal VARCHAR(20) CHECK (learning_goal IN ('career', 'research', 'hobby')),
            preferred_language VARCHAR(10) CHECK (preferred_language IN ('python', 'cpp', 'both')),
            content_language VARCHAR(5) DEFAULT 'en' CHECK (content_language IN ('en', 'ur')),
            profile_hash VARCHAR(64),
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
          );
          CREATE UNIQUE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);
          CREATE INDEX IF NOT EXISTS idx_user_profiles_hash ON user_profiles(profile_hash);
        `
      },
      chat_history: {
        description: 'Chat conversation history for RAG chatbot',
        sql: `
          CREATE TABLE IF NOT EXISTS chat_history (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            user_id UUID REFERENCES users(id) ON DELETE CASCADE,
            chat_id UUID NOT NULL,
            role VARCHAR(20) CHECK (role IN ('user', 'assistant', 'system')),
            content TEXT NOT NULL,
            sources JSONB,
            tokens_used INTEGER,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
          );
          CREATE INDEX IF NOT EXISTS idx_chat_history_user_id ON chat_history(user_id);
          CREATE INDEX IF NOT EXISTS idx_chat_history_chat_id ON chat_history(chat_id);
          CREATE INDEX IF NOT EXISTS idx_chat_history_created_at ON chat_history(created_at DESC);
        `
      },
      personalization_cache: {
        description: 'Cached personalized content',
        sql: `
          CREATE TABLE IF NOT EXISTS personalization_cache (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            chapter_id VARCHAR(255) NOT NULL,
            profile_hash VARCHAR(64) NOT NULL,
            content TEXT NOT NULL,
            metadata JSONB,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
            expires_at TIMESTAMP WITH TIME ZONE
          );
          CREATE UNIQUE INDEX IF NOT EXISTS idx_personalization_cache_key ON personalization_cache(chapter_id, profile_hash);
          CREATE INDEX IF NOT EXISTS idx_personalization_cache_expires ON personalization_cache(expires_at);
        `
      },
      translation_cache: {
        description: 'Cached translations',
        sql: `
          CREATE TABLE IF NOT EXISTS translation_cache (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            chapter_id VARCHAR(255) NOT NULL,
            source_language VARCHAR(5) NOT NULL,
            target_language VARCHAR(5) NOT NULL,
            content_hash VARCHAR(64) NOT NULL,
            translated_content TEXT NOT NULL,
            created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
          );
          CREATE UNIQUE INDEX IF NOT EXISTS idx_translation_cache_key ON translation_cache(chapter_id, target_language, content_hash);
        `
      },
      schema_migrations: {
        description: 'Migration tracking table',
        sql: `
          CREATE TABLE IF NOT EXISTS schema_migrations (
            version VARCHAR(255) PRIMARY KEY,
            name VARCHAR(255) NOT NULL,
            applied_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
          );
        `
      }
    };
  }

  /**
   * Connect to database
   */
  async connect() {
    if (this.pool) return;

    const connectionString = process.env.DATABASE_URL;
    if (!connectionString) {
      throw new Error('DATABASE_URL environment variable is required');
    }

    this.pool = new Pool({
      connectionString,
      max: parseInt(process.env.DATABASE_POOL_MAX || '10'),
      ssl: connectionString.includes('neon.tech') ? { rejectUnauthorized: false } : false
    });
  }

  /**
   * Disconnect from database
   */
  async disconnect() {
    if (this.pool) {
      await this.pool.end();
      this.pool = null;
    }
  }

  /**
   * Create a new migration file
   */
  async createMigration(options = {}) {
    const { name, tables = ['all'] } = options;

    if (!name) {
      throw new Error('Migration name is required');
    }

    // Ensure migrations directory exists
    if (!fs.existsSync(this.migrationsDir)) {
      fs.mkdirSync(this.migrationsDir, { recursive: true });
    }

    const timestamp = new Date().toISOString().replace(/[-:T.Z]/g, '').slice(0, 14);
    const slug = name.toLowerCase().replace(/[^a-z0-9]+/g, '_');
    const version = `${timestamp}_${slug}`;
    const filename = `${version}.sql`;
    const filepath = path.join(this.migrationsDir, filename);

    // Determine which tables to include
    const tablesToInclude = tables.includes('all')
      ? Object.keys(this.schemas).filter(t => t !== 'schema_migrations')
      : tables;

    // Generate migration content
    const upSql = [];
    const downSql = [];

    for (const table of tablesToInclude) {
      if (this.schemas[table]) {
        upSql.push(`-- Create ${table} table`);
        upSql.push(this.schemas[table].sql.trim());
        upSql.push('');

        downSql.push(`DROP TABLE IF EXISTS ${table} CASCADE;`);
      }
    }

    const content = `-- Migration: ${name}
-- Version: ${version}
-- Created: ${new Date().toISOString()}

-- ==================== UP ====================

${upSql.join('\n')}

-- ==================== DOWN ====================
-- To rollback, run the following SQL:
-- ${downSql.join('\n-- ')}
`;

    fs.writeFileSync(filepath, content, 'utf-8');

    return {
      success: true,
      migration_files: [filepath],
      version,
      tables: tablesToInclude
    };
  }

  /**
   * Run pending migrations
   */
  async runMigrations(options = {}) {
    await this.connect();

    try {
      // Ensure schema_migrations table exists
      await this.pool.query(this.schemas.schema_migrations.sql);

      // Get applied migrations
      const appliedResult = await this.pool.query(
        'SELECT version FROM schema_migrations ORDER BY version'
      );
      const appliedVersions = new Set(appliedResult.rows.map(r => r.version));

      // Get pending migrations
      if (!fs.existsSync(this.migrationsDir)) {
        return {
          success: true,
          migrations_applied: [],
          message: 'No migrations directory found'
        };
      }

      const migrationFiles = fs.readdirSync(this.migrationsDir)
        .filter(f => f.endsWith('.sql'))
        .sort();

      const migrationsApplied = [];

      for (const file of migrationFiles) {
        const version = file.replace('.sql', '');

        if (appliedVersions.has(version)) {
          continue;
        }

        const filepath = path.join(this.migrationsDir, file);
        const content = fs.readFileSync(filepath, 'utf-8');

        // Extract UP section
        const upMatch = content.match(/-- ==================== UP ====================\n([\s\S]*?)(?:-- ==================== DOWN ====================|$)/);
        if (!upMatch) {
          throw new Error(`Invalid migration format: ${file}`);
        }

        const upSql = upMatch[1].trim();

        // Execute migration
        await this.pool.query('BEGIN');
        try {
          await this.pool.query(upSql);
          await this.pool.query(
            'INSERT INTO schema_migrations (version, name) VALUES ($1, $2)',
            [version, file]
          );
          await this.pool.query('COMMIT');

          migrationsApplied.push({
            version,
            name: file,
            applied_at: new Date().toISOString()
          });

          console.log(`Applied migration: ${file}`);
        } catch (error) {
          await this.pool.query('ROLLBACK');
          throw new Error(`Migration ${file} failed: ${error.message}`);
        }
      }

      return {
        success: true,
        migrations_applied: migrationsApplied,
        current_version: migrationsApplied.length > 0
          ? migrationsApplied[migrationsApplied.length - 1].version
          : (appliedVersions.size > 0 ? [...appliedVersions].pop() : null)
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Rollback to a specific version
   */
  async rollback(options = {}) {
    const { targetVersion } = options;

    await this.connect();

    try {
      // Get applied migrations after target
      const query = targetVersion
        ? 'SELECT version, name FROM schema_migrations WHERE version > $1 ORDER BY version DESC'
        : 'SELECT version, name FROM schema_migrations ORDER BY version DESC LIMIT 1';

      const result = await this.pool.query(
        query,
        targetVersion ? [targetVersion] : []
      );

      if (result.rows.length === 0) {
        return {
          success: true,
          message: 'No migrations to rollback'
        };
      }

      const rolledBack = [];

      for (const row of result.rows) {
        const filepath = path.join(this.migrationsDir, `${row.version}.sql`);

        if (!fs.existsSync(filepath)) {
          console.warn(`Migration file not found: ${filepath}`);
          continue;
        }

        // Remove from tracking
        await this.pool.query(
          'DELETE FROM schema_migrations WHERE version = $1',
          [row.version]
        );

        rolledBack.push({
          version: row.version,
          name: row.name
        });

        console.log(`Rolled back: ${row.name}`);
      }

      return {
        success: true,
        rolled_back: rolledBack
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Get migration status
   */
  async status() {
    await this.connect();

    try {
      // Ensure schema_migrations table exists
      await this.pool.query(this.schemas.schema_migrations.sql);

      const appliedResult = await this.pool.query(
        'SELECT version, name, applied_at FROM schema_migrations ORDER BY version'
      );

      const pendingMigrations = [];
      if (fs.existsSync(this.migrationsDir)) {
        const appliedVersions = new Set(appliedResult.rows.map(r => r.version));
        const migrationFiles = fs.readdirSync(this.migrationsDir)
          .filter(f => f.endsWith('.sql'))
          .sort();

        for (const file of migrationFiles) {
          const version = file.replace('.sql', '');
          if (!appliedVersions.has(version)) {
            pendingMigrations.push({ version, name: file });
          }
        }
      }

      return {
        success: true,
        applied: appliedResult.rows,
        pending: pendingMigrations,
        current_version: appliedResult.rows.length > 0
          ? appliedResult.rows[appliedResult.rows.length - 1].version
          : null
      };
    } finally {
      await this.disconnect();
    }
  }

  /**
   * Generate complete schema file
   */
  async generateSchema(options = {}) {
    const { outputPath = 'schema.sql' } = options;

    const lines = [
      '-- Complete Database Schema',
      `-- Generated: ${new Date().toISOString()}`,
      '',
      '-- Enable UUID extension',
      'CREATE EXTENSION IF NOT EXISTS "pgcrypto";',
      ''
    ];

    for (const [name, schema] of Object.entries(this.schemas)) {
      lines.push(`-- ==================== ${name.toUpperCase()} ====================`);
      lines.push(`-- ${schema.description}`);
      lines.push(schema.sql.trim());
      lines.push('');
    }

    const fullPath = path.resolve(this.projectRoot, outputPath);
    fs.writeFileSync(fullPath, lines.join('\n'), 'utf-8');

    return {
      success: true,
      schema_file: fullPath
    };
  }
}

/**
 * CLI interface
 */
async function main() {
  const args = process.argv.slice(2);

  if (args.length < 1) {
    console.error('Usage: db-migrations <operation> [options]');
    console.error('Operations: create_migration, run_migrations, rollback, status, generate_schema');
    console.error('');
    console.error('Examples:');
    console.error('  db-migrations create_migration --name "initial_schema"');
    console.error('  db-migrations run_migrations');
    console.error('  db-migrations status');
    console.error('  db-migrations rollback --version "20240101000000"');
    console.error('  db-migrations generate_schema --output schema.sql');
    process.exit(1);
  }

  const operation = args[0];
  const options = parseCliArgs(args.slice(1));

  const migrations = new DbMigrations({ projectRoot: process.cwd() });

  try {
    let result;
    switch (operation) {
      case 'create_migration':
        result = await migrations.createMigration({
          name: options.name,
          tables: options.tables ? options.tables.split(',') : ['all']
        });
        break;

      case 'run_migrations':
        result = await migrations.runMigrations();
        break;

      case 'rollback':
        result = await migrations.rollback({
          targetVersion: options.version
        });
        break;

      case 'status':
        result = await migrations.status();
        break;

      case 'generate_schema':
        result = await migrations.generateSchema({
          outputPath: options.output || 'schema.sql'
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

/**
 * Parse CLI arguments
 */
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

// Export for use as a module
export default DbMigrations;
export { DbMigrations };

// Run CLI if called directly
const isMainModule = import.meta.url === `file://${process.argv[1]}` ||
                     process.argv[1]?.endsWith('db-migrations/index.js');
if (isMainModule) {
  main().catch(console.error);
}
