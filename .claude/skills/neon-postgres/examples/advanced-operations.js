/**
 * Advanced Operations Example
 * Demonstrates complex queries, transactions, and security features
 */

import { NeonPostgresSkill, SecurityManager } from '../index.js';

async function advancedOperations() {
  const db = new NeonPostgresSkill();
  const security = new SecurityManager(db.pool);

  try {
    await db.initialize();

    // 1. Complex SELECT with filters and ordering
    console.log('🔍 Complex SELECT query...');
    const usersWithProfiles = await db.execute({
      operation: 'select',
      table_name: 'user_summary', // Using the view
      query: {
        experience_level: 'intermediate',
        preferred_language: 'python'
      },
      orderBy: 'user_created_at DESC',
      limit: 10
    });
    console.log(`Found ${usersWithProfiles.data.length} intermediate Python users`);

    // 2. Transaction example - Update user and profile atomically
    console.log('\n🔄 Running transaction...');
    const result = await db.db.transaction(async (client) => {
      // Update user email
      const userUpdate = await client.query(
        'UPDATE users SET email = $1 WHERE id = $2 RETURNING *',
        ['updated.email@example.com', 'user-uuid-here']
      );

      // Update profile experience
      const profileUpdate = await client.query(
        'UPDATE user_profiles SET experience_level = $1 WHERE user_id = $2 RETURNING *',
        ['advanced', 'user-uuid-here']
      );

      return { user: userUpdate.rows[0], profile: profileUpdate.rows[0] };
    });
    console.log('✅ Transaction completed');

    // 3. Rate limiting example
    console.log('\n⏱️ Rate limiting check...');
    const rateLimit = await security.checkRateLimit('api-key-123', 5, 60000);
    if (rateLimit.allowed) {
      console.log(`✅ Request allowed. Remaining: ${rateLimit.remaining}`);
    } else {
      console.log(`❌ Rate limit exceeded. Reset at: ${rateLimit.resetTime}`);
    }

    // 4. Audit logging
    console.log('\n📝 Logging audit event...');
    await security.logAuditEvent('DATA_ACCESS', 'user-uuid-here', {
      tableName: 'chat_history',
      recordId: 'chat-uuid-here',
      ipAddress: '192.168.1.100',
      userAgent: 'Mozilla/5.0...',
      oldValues: { message: 'Old message' },
      newValues: { message: 'New message' }
    });
    console.log('✅ Audit event logged');

    // 5. Advanced chat queries
    console.log('\n💬 Advanced chat queries...');

    // Get chat sessions with message counts
    const sessionStats = await db.db.query(`
      SELECT
        session_id,
        user_id,
        COUNT(*) as message_count,
        MIN(created_at) as first_message,
        MAX(created_at) as last_message
      FROM chat_history
      WHERE created_at > NOW() - INTERVAL '7 days'
      GROUP BY session_id, user_id
      ORDER BY last_message DESC
      LIMIT 10
    `);

    console.log(`Active sessions (last 7 days): ${sessionStats.rows.length}`);
    sessionStats.rows.forEach(session => {
      console.log(`  Session ${session.session_id.substring(0, 8)}: ${session.message_count} messages`);
    });

    // 6. Cache cleanup
    console.log('\n🧹 Cleaning expired cache...');
    const cleanupResult = await db.personalizationCache.clearExpired();
    console.log(`✅ Cleaned ${cleanupResult.deletedCount} expired cache entries`);

    // 7. Batch operations
    console.log('\n📦 Batch insert example...');
    const batchData = [
      { chapter_id: 'chapter-1', user_profile_hash: 'hash1', adapted_content: 'Content 1', expires_at: new Date(Date.now() + 86400000) },
      { chapter_id: 'chapter-2', user_profile_hash: 'hash2', adapted_content: 'Content 2', expires_at: new Date(Date.now() + 86400000) },
      { chapter_id: 'chapter-3', user_profile_hash: 'hash3', adapted_content: 'Content 3', expires_at: new Date(Date.now() + 86400000) }
    ];

    const batchResult = await db.db.transaction(async (client) => {
      const results = [];
      for (const data of batchData) {
        const result = await client.query(`
          INSERT INTO personalization_cache (chapter_id, user_profile_hash, adapted_content, expires_at)
          VALUES ($1, $2, $3, $4)
          RETURNING *
        `, [data.chapter_id, data.user_profile_hash, data.adapted_content, data.expires_at]);
        results.push(result.rows[0]);
      }
      return results;
    });
    console.log(`✅ Inserted ${batchResult.length} cache entries`);

    // 8. Analytics query
    console.log('\n📊 User analytics...');
    const analytics = await db.db.query(`
      SELECT
        up.experience_level,
        up.ros_familiarity,
        COUNT(*) as user_count,
        AVG(ch.message_count) as avg_messages
      FROM user_profiles up
      LEFT JOIN (
        SELECT
          user_id,
          COUNT(*) as message_count
        FROM chat_history
        WHERE created_at > NOW() - INTERVAL '30 days'
        GROUP BY user_id
      ) ch ON up.user_id = ch.user_id
      GROUP BY up.experience_level, up.ros_familiarity
      ORDER BY user_count DESC
    `);

    console.log('User segments:');
    analytics.rows.forEach(row => {
      console.log(`  ${row.experience_level}/${row.ros_familiarity}: ${row.user_count} users, ${Math.round(row.avg_messages || 0)} avg messages`);
    });

    // 9. Security operations
    console.log('\n🔐 Security operations...');

    // Set user context for RLS
    await security.setUserContext('user-uuid-here');

    // This query will only return data for the set user
    const userSpecificData = await db.execute({
      operation: 'select',
      table_name: 'chat_history'
    });

    // Clear user context
    await security.clearUserContext();

    console.log(`✅ Retrieved ${userSpecificData.data.length} user-specific records`);

    // 10. Performance monitoring
    console.log('\n⚡ Performance metrics...');
    const slowQueries = await db.db.query(`
      SELECT
        query,
        calls,
        total_time,
        mean_time,
        rows
      FROM pg_stat_statements
      WHERE mean_time > 1000
      ORDER BY mean_time DESC
      LIMIT 5
    `);

    if (slowQueries.rows.length > 0) {
      console.log('Slow queries found:');
      slowQueries.rows.forEach(q => {
        console.log(`  ${q.query.substring(0, 50)}...: ${Math.round(q.mean_time)}ms avg`);
      });
    } else {
      console.log('✅ No slow queries detected');
    }

  } catch (error) {
    console.error('❌ Error in advanced operations:', error);
  } finally {
    await db.close();
  }
}

// Run the example
if (import.meta.url === `file://${process.argv[1]}`) {
  advancedOperations();
}

export { advancedOperations };