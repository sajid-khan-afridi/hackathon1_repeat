/**
 * Security utilities and audit logging
 */

import crypto from 'crypto';

export class SecurityManager {
  constructor(db) {
    this.db = db;
  }

  /**
   * Hash password using bcrypt-compatible method
   * Note: In production, use bcrypt library instead
   */
  hashPassword(password) {
    // For demonstration - use bcrypt in production
    const salt = crypto.randomBytes(16).toString('hex');
    const hash = crypto.pbkdf2Sync(password, salt, 10000, 64, 'sha512').toString('hex');
    return `${salt}:${hash}`;
  }

  /**
   * Verify password
   */
  verifyPassword(password, hashedPassword) {
    const [salt, hash] = hashedPassword.split(':');
    const verifyHash = crypto.pbkdf2Sync(password, salt, 10000, 64, 'sha512').toString('hex');
    return hash === verifyHash;
  }

  /**
   * Generate API token
   */
  generateApiToken(userId, expiresIn = '24h') {
    const payload = {
      sub: userId,
      iat: Math.floor(Date.now() / 1000),
      exp: Math.floor(Date.now() / 1000) + (60 * 60 * 24) // 24 hours
    };

    // For demonstration - use JWT library in production
    const token = Buffer.from(JSON.stringify(payload)).toString('base64');
    return token;
  }

  /**
   * Set user context for RLS policies
   */
  async setUserContext(userId) {
    await this.db.query('SET app.current_user_id = $1', [userId]);
  }

  /**
   * Clear user context
   */
  async clearUserContext() {
    await this.db.query('RESET app.current_user_id');
  }

  /**
   * Create audit log entry
   */
  async logAuditEvent(eventType, userId, details = {}) {
    const auditLogSQL = `
      CREATE TABLE IF NOT EXISTS audit_log (
        id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
        event_type VARCHAR(50) NOT NULL,
        user_id UUID,
        table_name VARCHAR(50),
        record_id UUID,
        old_values JSONB,
        new_values JSONB,
        ip_address INET,
        user_agent TEXT,
        created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
      );
    `;

    await this.db.query(auditLogSQL);

    const insertSQL = `
      INSERT INTO audit_log (event_type, user_id, table_name, record_id, old_values, new_values, ip_address, user_agent)
      VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
      RETURNING *
    `;

    const params = [
      eventType,
      userId,
      details.tableName || null,
      details.recordId || null,
      details.oldValues || null,
      details.newValues || null,
      details.ipAddress || null,
      details.userAgent || null
    ];

    return await this.db.query(insertSQL, params);
  }

  /**
   * Sanitize input to prevent SQL injection
   */
  sanitizeInput(input) {
    if (typeof input === 'string') {
      // Basic SQL injection prevention
      return input.replace(/[';\\]/g, '');
    }
    return input;
  }

  /**
   * Validate email format
   */
  validateEmail(email) {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  }

  /**
   * Validate UUID format
   */
  validateUUID(uuid) {
    const uuidRegex = /^[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
    return uuidRegex.test(uuid);
  }

  /**
   * Rate limiting check
   */
  async checkRateLimit(identifier, maxRequests = 100, windowMs = 60000) {
    const rateLimitSQL = `
      CREATE TABLE IF NOT EXISTS rate_limits (
        identifier VARCHAR(255) PRIMARY KEY,
        requests INTEGER DEFAULT 1,
        window_start TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
        updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
      );
    `;

    await this.db.query(rateLimitSQL);

    // Check existing rate limit
    const existing = await this.db.query(
      'SELECT * FROM rate_limits WHERE identifier = $1 AND window_start > NOW() - INTERVAL \'1 millisecond\' * $2',
      [identifier, windowMs]
    );

    if (existing.rows.length > 0) {
      const current = existing.rows[0];
      if (current.requests >= maxRequests) {
        return {
          allowed: false,
          remaining: 0,
          resetTime: new Date(Date.parse(current.window_start) + windowMs)
        };
      }

      // Increment counter
      await this.db.query(
        'UPDATE rate_limits SET requests = requests + 1, updated_at = CURRENT_TIMESTAMP WHERE identifier = $1',
        [identifier]
      );

      return {
        allowed: true,
        remaining: maxRequests - current.requests - 1,
        resetTime: new Date(Date.parse(current.window_start) + windowMs)
      };
    }

    // Create new rate limit entry
    await this.db.query(
      'INSERT INTO rate_limits (identifier, requests, window_start) VALUES ($1, 1, CURRENT_TIMESTAMP)',
      [identifier]
    );

    return {
      allowed: true,
      remaining: maxRequests - 1,
      resetTime: new Date(Date.now() + windowMs)
    };
  }

  /**
   * Clean expired rate limit entries
   */
  async cleanExpiredRateLimits() {
    await this.db.query(
      'DELETE FROM rate_limits WHERE window_start < NOW() - INTERVAL \'1 hour\''
    );
  }

  /**
   * Encrypt sensitive data (for storage)
   */
  encrypt(data, key) {
    const algorithm = 'aes-256-gcm';
    const iv = crypto.randomBytes(16);
    const cipher = crypto.createCipher(algorithm, key, iv);

    let encrypted = cipher.update(JSON.stringify(data), 'utf8', 'hex');
    encrypted += cipher.final('hex');

    const authTag = cipher.getAuthTag();

    return {
      encrypted,
      iv: iv.toString('hex'),
      authTag: authTag.toString('hex')
    };
  }

  /**
   * Decrypt sensitive data
   */
  decrypt(encryptedData, key) {
    const algorithm = 'aes-256-gcm';
    const decipher = crypto.createDecipher(
      algorithm,
      key,
      Buffer.from(encryptedData.iv, 'hex')
    );

    decipher.setAuthTag(Buffer.from(encryptedData.authTag, 'hex'));

    let decrypted = decipher.update(encryptedData.encrypted, 'hex', 'utf8');
    decrypted += decipher.final('utf8');

    return JSON.parse(decrypted);
  }
}

/**
 * Audit middleware for tracking database operations
 */
export function createAuditMiddleware(securityManager) {
  return async (operation, tableName, userId, details = {}) => {
    const eventType = `${operation.toUpperCase()}_${tableName.toUpperCase()}`;
    await securityManager.logAuditEvent(eventType, userId, {
      tableName,
      ...details
    });
  };
}