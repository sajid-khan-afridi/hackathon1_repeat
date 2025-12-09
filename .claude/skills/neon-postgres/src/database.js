/**
 * Database Connection and Pool Management
 */

import pg from 'pg';
import { v4 as uuidv4 } from 'uuid';

export class DatabaseManager {
  constructor(config = {}) {
    this.config = {
      // Default configuration with connection pooling
      connectionString: process.env.NEON_DATABASE_URL || config.connectionString,
      ssl: { rejectUnauthorized: false }, // Required for Neon
      max: config.maxConnections || 10, // Neon free tier limit
      idleTimeoutMillis: config.idleTimeout || 30000,
      connectionTimeoutMillis: config.connectionTimeout || 30000,
      ...config
    };

    this.pool = null;
    this.isConnected = false;
  }

  /**
   * Initialize database connection pool
   */
  async connect() {
    if (this.isConnected) {
      return;
    }

    try {
      this.pool = new pg.Pool(this.config);

      // Test the connection
      const client = await this.pool.connect();
      const result = await client.query('SELECT NOW()');
      client.release();

      this.isConnected = true;
      console.log('Connected to Neon PostgreSQL database');
      return result;
    } catch (error) {
      console.error('Failed to connect to database:', error);
      throw new Error(`Database connection failed: ${error.message}`);
    }
  }

  /**
   * Close all connections in the pool
   */
  async disconnect() {
    if (this.pool) {
      await this.pool.end();
      this.pool = null;
      this.isConnected = false;
      console.log('Database connection pool closed');
    }
  }

  /**
   * Execute a query with automatic retry logic
   * @param {string} text - SQL query
   * @param {Array} params - Query parameters
   * @param {number} retries - Number of retry attempts
   * @returns {Promise<Object>} - Query result
   */
  async query(text, params = [], retries = 3) {
    if (!this.isConnected) {
      await this.connect();
    }

    const startTime = Date.now();
    let attempt = 0;

    while (attempt < retries) {
      let client;
      try {
        client = await this.pool.connect();
        const result = await client.query(text, params);
        client.release();

        // Log slow queries
        const duration = Date.now() - startTime;
        if (duration > 1000) {
          console.warn(`Slow query detected (${duration}ms):`, text);
        }

        return result;
      } catch (error) {
        if (client) client.release();
        attempt++;

        // Retry on timeout or connection errors
        if ((error.code === 'ECONNRESET' || error.code === 'ETIMEDOUT' || error.message.includes('timeout')) && attempt < retries) {
          console.warn(`Query attempt ${attempt} failed, retrying... Error:`, error.message);
          await this.delay(1000 * attempt); // Exponential backoff
          continue;
        }

        console.error('Query failed:', error);
        throw new Error(`Query execution failed: ${error.message}`);
      }
    }
  }

  /**
   * Execute a transaction with multiple queries
   * @param {Function} callback - Function that receives a client and returns queries
   * @returns {Promise<Object>} - Transaction result
   */
  async transaction(callback) {
    const client = await this.pool.connect();
    try {
      await client.query('BEGIN');
      const result = await callback(client);
      await client.query('COMMIT');
      return result;
    } catch (error) {
      await client.query('ROLLBACK');
      throw error;
    } finally {
      client.release();
    }
  }

  /**
   * Get database pool statistics
   */
  getPoolStats() {
    if (!this.pool) {
      return null;
    }

    return {
      totalCount: this.pool.totalCount,
      idleCount: this.pool.idleCount,
      waitingCount: this.pool.waitingCount
    };
  }

  /**
   * Check database health
   */
  async healthCheck() {
    try {
      const result = await this.query('SELECT 1 as health');
      return { status: 'healthy', timestamp: new Date().toISOString() };
    } catch (error) {
      return {
        status: 'unhealthy',
        error: error.message,
        timestamp: new Date().toISOString()
      };
    }
  }

  /**
   * Utility function to add delay
   */
  delay(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  /**
   * Generate UUID v4
   */
  generateUUID() {
    return uuidv4();
  }
}