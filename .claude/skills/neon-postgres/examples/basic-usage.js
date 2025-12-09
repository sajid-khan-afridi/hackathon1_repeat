/**
 * Basic Usage Example
 * Demonstrates common database operations
 */

import { NeonPostgresSkill, SecurityManager } from '../index.js';

async function basicUsage() {
  // Initialize database connection
  const db = new NeonPostgresSkill({
    connectionString: process.env.NEON_DATABASE_URL,
    maxConnections: 5
  });

  const security = new SecurityManager(db.pool);

  try {
    console.log('🚀 Initializing database...');
    await db.initialize();

    // 1. Create a new user
    console.log('\n📝 Creating user...');
    const user = await db.users.create({
      email: 'john.doe@example.com',
      password_hash: security.hashPassword('securePassword123')
    });
    console.log('✅ User created:', user.data);

    // 2. Add user profile
    console.log('\n📊 Adding user profile...');
    await db.userProfiles.create({
      user_id: user.data.id,
      experience_level: 'intermediate',
      ros_familiarity: 'basic',
      hardware_access: 'simulation',
      learning_goal: 'career',
      preferred_language: 'python'
    });
    console.log('✅ User profile created');

    // 3. Start a chat session
    const sessionId = db.db.generateUUID();
    console.log('\n💬 Starting chat session:', sessionId);

    // Add user message
    await db.chatHistory.add({
      user_id: user.data.id,
      session_id: sessionId,
      message: 'How do I get started with ROS?',
      role: 'user'
    });
    console.log('✅ User message added');

    // Add assistant response
    await db.chatHistory.add({
      user_id: user.data.id,
      session_id: sessionId,
      message: 'Great question! To get started with ROS, I recommend...',
      role: 'assistant'
    });
    console.log('✅ Assistant response added');

    // 4. Retrieve chat history
    console.log('\n📜 Retrieving chat history...');
    const history = await db.chatHistory.getBySession(sessionId);
    console.log('Chat messages:');
    history.data.forEach(msg => {
      console.log(`  ${msg.role}: ${msg.message}`);
    });

    // 5. Cache personalized content
    console.log('\n🗄️ Caching personalized content...');
    const profileHash = 'hash-of-user-profile';
    await db.personalizationCache.set({
      chapter_id: 'ros-introduction',
      user_profile_hash: profileHash,
      adapted_content: 'ROS (Robot Operating System) is specially tailored for intermediate Python developers...',
      expires_at: new Date(Date.now() + 24 * 60 * 60 * 1000) // 24 hours
    });
    console.log('✅ Content cached');

    // 6. Retrieve cached content
    console.log('\n🔍 Retrieving cached content...');
    const cached = await db.personalizationCache.get('ros-introduction', profileHash);
    if (cached.data.length > 0) {
      console.log('Cached content:', cached.data[0].adapted_content.substring(0, 100) + '...');
    }

    // 7. Get user statistics
    console.log('\n📊 User statistics:');
    const chatCount = await db.chatHistory.getByUser(user.data.id);
    console.log(`  Total messages: ${chatCount.data.length}`);

    // 8. Update user profile
    console.log('\n✏️ Updating user experience level...');
    await db.userProfiles.update(user.data.id, {
      experience_level: 'advanced'
    });
    console.log('✅ Profile updated');

    // 9. Check database health
    console.log('\n🏥 Database health check...');
    const health = await db.db.healthCheck();
    console.log('Status:', health.status);

    // 10. Get pool statistics
    console.log('\n📊 Connection pool stats:');
    const stats = db.db.getPoolStats();
    console.log('  Total connections:', stats.totalCount);
    console.log('  Idle connections:', stats.idleCount);

  } catch (error) {
    console.error('❌ Error:', error.message);
  } finally {
    // Always close the connection
    await db.close();
    console.log('\n👋 Database connection closed');
  }
}

// Run the example
if (import.meta.url === `file://${process.argv[1]}`) {
  basicUsage();
}

export { basicUsage };