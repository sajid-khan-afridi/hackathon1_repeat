/**
 * Neon PostgreSQL Database Skill
 * Handles all database operations for user management, chat history, and personalization data
 */

import { DatabaseManager } from './src/database.js';
import { TableManager } from './src/tables.js';

class NeonPostgresSkill {
  constructor(config = {}) {
    this.db = new DatabaseManager(config);
    this.tables = new TableManager(this.db);
  }

  /**
   * Initialize the database connection and create tables if they don't exist
   */
  async initialize() {
    await this.db.connect();
    await this.tables.createAllTables();
  }

  /**
   * Close the database connection pool
   */
  async close() {
    await this.db.disconnect();
  }

  /**
   * Execute a database operation
   * @param {Object} params - Operation parameters
   * @param {string} params.operation - Type of operation (create_table, insert, select, update, delete, migrate)
   * @param {string} params.table_name - Target table name
   * @param {Object} [params.schema] - Schema definition for create_table
   * @param {Object} [params.data] - Data object for insert/update operations
   * @param {Object} [params.query] - Query filters for select operations
   * @param {Object} [params.where] - Where conditions for update/delete operations
   * @returns {Promise<Object>} - Operation result
   */
  async execute({ operation, table_name, schema, data, query, where }) {
    try {
      switch (operation) {
        case 'create_table':
          return await this.tables.createTable(table_name, schema);

        case 'insert':
          return await this.tables.insert(table_name, data);

        case 'select':
          return await this.tables.select(table_name, query);

        case 'update':
          return await this.tables.update(table_name, data, where);

        case 'delete':
          return await this.tables.delete(table_name, where);

        case 'migrate':
          return await this.tables.migrate();

        default:
          throw new Error(`Unknown operation: ${operation}`);
      }
    } catch (error) {
      console.error(`Database operation failed:`, error);
      throw error;
    }
  }

  /**
   * Helper method for user operations
   */
  get users() {
    return {
      create: (userData) => this.execute({ operation: 'insert', table_name: 'users', data: userData }),
      findByEmail: (email) => this.execute({ operation: 'select', table_name: 'users', query: { email } }),
      update: (userId, userData) => this.execute({
        operation: 'update',
        table_name: 'users',
        data: userData,
        where: { id: userId }
      }),
      delete: (userId) => this.execute({ operation: 'delete', table_name: 'users', where: { id: userId } })
    };
  }

  /**
   * Helper method for user profile operations
   */
  get userProfiles() {
    return {
      create: (profileData) => this.execute({ operation: 'insert', table_name: 'user_profiles', data: profileData }),
      findByUserId: (userId) => this.execute({ operation: 'select', table_name: 'user_profiles', query: { user_id: userId } }),
      update: (userId, profileData) => this.execute({
        operation: 'update',
        table_name: 'user_profiles',
        data: profileData,
        where: { user_id: userId }
      })
    };
  }

  /**
   * Helper method for chat history operations
   */
  get chatHistory() {
    return {
      add: (messageData) => this.execute({ operation: 'insert', table_name: 'chat_history', data: messageData }),
      getBySession: (sessionId) => this.execute({
        operation: 'select',
        table_name: 'chat_history',
        query: { session_id: sessionId },
        orderBy: 'created_at ASC'
      }),
      getByUser: (userId) => this.execute({
        operation: 'select',
        table_name: 'chat_history',
        query: { user_id: userId },
        orderBy: 'created_at DESC'
      }),
      deleteOldMessages: (beforeDate) => this.execute({
        operation: 'delete',
        table_name: 'chat_history',
        where: { created_at: { lt: beforeDate } }
      })
    };
  }

  /**
   * Helper method for personalization cache operations
   */
  get personalizationCache() {
    return {
      set: (cacheData) => this.execute({ operation: 'insert', table_name: 'personalization_cache', data: cacheData }),
      get: (chapterId, userProfileHash) => this.execute({
        operation: 'select',
        table_name: 'personalization_cache',
        query: { chapter_id: chapterId, user_profile_hash: userProfileHash }
      }),
      clearExpired: () => this.execute({
        operation: 'delete',
        table_name: 'personalization_cache',
        where: { expires_at: { lt: new Date() } }
      })
    };
  }
}

export { NeonPostgresSkill };
export default NeonPostgresSkill;