/**
 * Database Migration Management
 */

import { MIGRATIONS } from './schemas.js';

export class MigrationManager {
  constructor(db) {
    this.db = db;
  }

  /**
   * Initialize migration tracking table
   */
  async init() {
    const createMigrationTable = `
      CREATE TABLE IF NOT EXISTS schema_migrations (
        version VARCHAR(3) PRIMARY KEY,
        description TEXT,
        executed_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
        execution_time_ms INTEGER
      );
    `;
    await this.db.query(createMigrationTable);
  }

  /**
   * Get list of executed migrations
   */
  async getExecutedMigrations() {
    const result = await this.db.query('SELECT version FROM schema_migrations ORDER BY version');
    return result.rows.map(row => row.version);
  }

  /**
   * Execute pending migrations
   */
  async migrate() {
    await this.init();
    const executed = await this.getExecutedMigrations();

    for (const migration of MIGRATIONS) {
      if (!executed.includes(migration.version)) {
        console.log(`Running migration ${migration.version}: ${migration.description}`);
        const startTime = Date.now();

        try {
          await migration.up(this.db);
          const executionTime = Date.now() - startTime;

          // Record migration
          await this.db.query(
            'INSERT INTO schema_migrations (version, description, execution_time_ms) VALUES ($1, $2, $3)',
            [migration.version, migration.description, executionTime]
          );

          console.log(`Migration ${migration.version} completed in ${executionTime}ms`);
        } catch (error) {
          console.error(`Migration ${migration.version} failed:`, error);
          throw error;
        }
      }
    }
  }

  /**
   * Get migration status
   */
  async status() {
    await this.init();
    const executed = await this.getExecutedMigrations();
    const pending = MIGRATIONS.filter(m => !executed.includes(m.version));

    return {
      executed: executed.length,
      pending: pending.length,
      lastMigration: executed[executed.length - 1] || null,
      pendingMigrations: pending.map(m => ({
        version: m.version,
        description: m.description
      }))
    };
  }
}

/**
 * Run migrations when this file is executed directly
 */
export async function runMigrations() {
  const { DatabaseManager } = await import('./database.js');
  const db = new DatabaseManager();
  const migration = new MigrationManager(db);

  try {
    await db.connect();
    const status = await migration.status();
    console.log('Migration status:', status);

    if (status.pending > 0) {
      console.log(`Running ${status.pending} pending migrations...`);
      await migration.migrate();
      console.log('All migrations completed successfully!');
    } else {
      console.log('Database is up to date');
    }
  } catch (error) {
    console.error('Migration failed:', error);
    process.exit(1);
  } finally {
    await db.disconnect();
  }
}