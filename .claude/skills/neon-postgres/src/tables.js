/**
 * CRUD Operations for Database Tables
 */

import { TABLE_SCHEMAS } from './schemas.js';
import { MigrationManager } from './migration.js';

export class TableManager {
  constructor(db) {
    this.db = db;
    this.migration = new MigrationManager(db);
  }

  /**
   * Create a table with the given schema
   */
  async createTable(tableName, schema = null) {
    const tableSchema = schema || TABLE_SCHEMAS[tableName];

    if (!tableSchema) {
      throw new Error(`No schema found for table: ${tableName}`);
    }

    const columns = Object.entries(tableSchema)
      .map(([col, def]) => `${col} ${def}`)
      .join(', ');

    const createTableSQL = `CREATE TABLE IF NOT EXISTS ${tableName} (${columns});`;
    await this.db.query(createTableSQL);

    return {
      success: true,
      table: tableName,
      message: `Table ${tableName} created successfully`
    };
  }

  /**
   * Create all tables defined in schemas
   */
  async createAllTables() {
    const results = [];

    for (const tableName of Object.keys(TABLE_SCHEMAS)) {
      try {
        const result = await this.createTable(tableName);
        results.push(result);
      } catch (error) {
        results.push({
          success: false,
          table: tableName,
          error: error.message
        });
      }
    }

    return results;
  }

  /**
   * Run all database migrations
   */
  async migrate() {
    return await this.migration.migrate();
  }

  /**
   * Insert data into a table
   */
  async insert(tableName, data) {
    if (!data || typeof data !== 'object') {
      throw new Error('Data must be a valid object');
    }

    const columns = Object.keys(data);
    const values = Object.values(data);
    const placeholders = columns.map((_, index) => `$${index + 1}`).join(', ');

    const insertSQL = `
      INSERT INTO ${tableName} (${columns.join(', ')})
      VALUES (${placeholders})
      RETURNING *
    `;

    const result = await this.db.query(insertSQL, values);

    return {
      success: true,
      data: result.rows[0],
      message: `Record inserted into ${tableName}`
    };
  }

  /**
   * Select records from a table
   */
  async select(tableName, query = {}, options = {}) {
    let selectSQL = `SELECT * FROM ${tableName}`;
    const params = [];
    const conditions = [];
    let paramIndex = 1;

    // Build WHERE clause
    if (query && Object.keys(query).length > 0) {
      for (const [key, value] of Object.entries(query)) {
        if (typeof value === 'object' && value !== null) {
          // Handle operators like { gt: 100, lt: 200 }
          for (const [op, opValue] of Object.entries(value)) {
            switch (op) {
              case 'gt':
                conditions.push(`${key} > $${paramIndex}`);
                params.push(opValue);
                paramIndex++;
                break;
              case 'gte':
                conditions.push(`${key} >= $${paramIndex}`);
                params.push(opValue);
                paramIndex++;
                break;
              case 'lt':
                conditions.push(`${key} < $${paramIndex}`);
                params.push(opValue);
                paramIndex++;
                break;
              case 'lte':
                conditions.push(`${key} <= $${paramIndex}`);
                params.push(opValue);
                paramIndex++;
                break;
              case 'like':
                conditions.push(`${key} LIKE $${paramIndex}`);
                params.push(opValue);
                paramIndex++;
                break;
              case 'in':
                if (Array.isArray(opValue)) {
                  const placeholders = opValue.map((_, idx) => `$${paramIndex + idx}`).join(', ');
                  conditions.push(`${key} IN (${placeholders})`);
                  params.push(...opValue);
                  paramIndex += opValue.length;
                }
                break;
              default:
                throw new Error(`Unknown operator: ${op}`);
            }
          }
        } else {
          conditions.push(`${key} = $${paramIndex}`);
          params.push(value);
          paramIndex++;
        }
      }

      if (conditions.length > 0) {
        selectSQL += ` WHERE ${conditions.join(' AND ')}`;
      }
    }

    // Add ORDER BY
    if (options.orderBy) {
      selectSQL += ` ORDER BY ${options.orderBy}`;
    }

    // Add LIMIT
    if (options.limit) {
      selectSQL += ` LIMIT $${paramIndex}`;
      params.push(parseInt(options.limit));
    }

    // Add OFFSET
    if (options.offset) {
      selectSQL += ` OFFSET $${paramIndex + 1}`;
      params.push(parseInt(options.offset));
    }

    const result = await this.db.query(selectSQL, params);

    return {
      success: true,
      data: result.rows,
      count: result.rows.length,
      message: `Retrieved ${result.rows.length} records from ${tableName}`
    };
  }

  /**
   * Update records in a table
   */
  async update(tableName, data, where) {
    if (!data || typeof data !== 'object' || Object.keys(data).length === 0) {
      throw new Error('Update data must be a non-empty object');
    }

    if (!where || typeof where !== 'object' || Object.keys(where).length === 0) {
      throw new Error('WHERE clause is required for update operations');
    }

    const updateClauses = [];
    const whereClauses = [];
    const params = [];
    let paramIndex = 1;

    // Build SET clause
    for (const [key, value] of Object.entries(data)) {
      updateClauses.push(`${key} = $${paramIndex}`);
      params.push(value);
      paramIndex++;
    }

    // Build WHERE clause
    for (const [key, value] of Object.entries(where)) {
      whereClauses.push(`${key} = $${paramIndex}`);
      params.push(value);
      paramIndex++;
    }

    const updateSQL = `
      UPDATE ${tableName}
      SET ${updateClauses.join(', ')}
      WHERE ${whereClauses.join(' AND ')}
      RETURNING *
    `;

    const result = await this.db.query(updateSQL, params);

    return {
      success: true,
      data: result.rows,
      count: result.rows.length,
      message: `Updated ${result.rows.length} records in ${tableName}`
    };
  }

  /**
   * Delete records from a table
   */
  async delete(tableName, where) {
    if (!where || typeof where !== 'object' || Object.keys(where).length === 0) {
      throw new Error('WHERE clause is required for delete operations');
    }

    const whereClauses = [];
    const params = [];
    let paramIndex = 1;

    // Build WHERE clause
    for (const [key, value] of Object.entries(where)) {
      whereClauses.push(`${key} = $${paramIndex}`);
      params.push(value);
      paramIndex++;
    }

    const deleteSQL = `
      DELETE FROM ${tableName}
      WHERE ${whereClauses.join(' AND ')}
      RETURNING *
    `;

    const result = await this.db.query(deleteSQL, params);

    return {
      success: true,
      deletedCount: result.rows.length,
      message: `Deleted ${result.rows.length} records from ${tableName}`
    };
  }

  /**
   * Check if a record exists
   */
  async exists(tableName, where) {
    const selectSQL = `SELECT 1 FROM ${tableName} WHERE `;
    const conditions = [];
    const params = [];
    let paramIndex = 1;

    for (const [key, value] of Object.entries(where)) {
      conditions.push(`${key} = $${paramIndex}`);
      params.push(value);
      paramIndex++;
    }

    const result = await this.db.query(selectSQL + conditions.join(' AND '), params);

    return {
      exists: result.rows.length > 0,
      count: result.rows.length
    };
  }

  /**
   * Get count of records
   */
  async count(tableName, where = {}) {
    let countSQL = `SELECT COUNT(*) as count FROM ${tableName}`;
    const params = [];
    const conditions = [];

    if (Object.keys(where).length > 0) {
      let paramIndex = 1;
      for (const [key, value] of Object.entries(where)) {
        conditions.push(`${key} = $${paramIndex}`);
        params.push(value);
        paramIndex++;
      }
      countSQL += ` WHERE ${conditions.join(' AND ')}`;
    }

    const result = await this.db.query(countSQL, params);

    return {
      count: parseInt(result.rows[0].count),
      message: `Found ${result.rows[0].count} records in ${tableName}`
    };
  }

  /**
   * Get table schema information
   */
  async getTableInfo(tableName) {
    const infoSQL = `
      SELECT
        column_name,
        data_type,
        is_nullable,
        column_default,
        character_maximum_length
      FROM information_schema.columns
      WHERE table_name = $1
      ORDER BY ordinal_position;
    `;

    const result = await this.db.query(infoSQL, [tableName]);

    return {
      table: tableName,
      columns: result.rows
    };
  }
}