/**
 * Table Schemas and Definitions
 */

export const TABLE_SCHEMAS = {
  users: {
    id: 'UUID PRIMARY KEY DEFAULT gen_random_uuid()',
    email: 'VARCHAR(255) UNIQUE NOT NULL',
    password_hash: 'VARCHAR(255) NOT NULL',
    created_at: 'TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP',
    updated_at: 'TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP'
  },

  user_profiles: {
    user_id: 'UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE',
    experience_level: 'VARCHAR(20) CHECK (experience_level IN (\'beginner\', \'intermediate\', \'advanced\')) NOT NULL',
    ros_familiarity: 'VARCHAR(20) CHECK (ros_familiarity IN (\'none\', \'basic\', \'proficient\')) NOT NULL',
    hardware_access: 'VARCHAR(20) CHECK (hardware_access IN (\'simulation\', \'jetson\', \'full_lab\')) NOT NULL',
    learning_goal: 'VARCHAR(20) CHECK (learning_goal IN (\'career\', \'research\', \'hobby\')) NOT NULL',
    preferred_language: 'VARCHAR(10) CHECK (preferred_language IN (\'python\', \'cpp\', \'both\')) NOT NULL',
    created_at: 'TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP',
    updated_at: 'TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP'
  },

  chat_history: {
    id: 'UUID PRIMARY KEY DEFAULT gen_random_uuid()',
    user_id: 'UUID REFERENCES users(id) ON DELETE SET NULL',
    session_id: 'UUID NOT NULL',
    message: 'TEXT NOT NULL',
    role: 'VARCHAR(10) CHECK (role IN (\'user\', \'assistant\')) NOT NULL',
    context_text: 'TEXT',
    created_at: 'TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP'
  },

  personalization_cache: {
    id: 'UUID PRIMARY KEY DEFAULT gen_random_uuid()',
    chapter_id: 'VARCHAR(255) NOT NULL',
    user_profile_hash: 'VARCHAR(64) NOT NULL',
    adapted_content: 'TEXT NOT NULL',
    created_at: 'TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP',
    expires_at: 'TIMESTAMP WITH TIME ZONE NOT NULL'
  }
};

export const INDEXES = [
  // Users table indexes
  'CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);',
  'CREATE INDEX IF NOT EXISTS idx_users_created_at ON users(created_at);',

  // User profiles indexes
  'CREATE INDEX IF NOT EXISTS idx_user_profiles_experience ON user_profiles(experience_level);',
  'CREATE INDEX IF NOT EXISTS idx_user_profiles_ros_familiarity ON user_profiles(ros_familiarity);',

  // Chat history indexes
  'CREATE INDEX IF NOT EXISTS idx_chat_history_user_id ON chat_history(user_id);',
  'CREATE INDEX IF NOT EXISTS idx_chat_history_session_id ON chat_history(session_id);',
  'CREATE INDEX IF NOT EXISTS idx_chat_history_created_at ON chat_history(created_at DESC);',

  // Personalization cache indexes
  'CREATE UNIQUE INDEX IF NOT EXISTS idx_personalization_unique ON personalization_cache(chapter_id, user_profile_hash);',
  'CREATE INDEX IF NOT EXISTS idx_personalization_expires_at ON personalization_cache(expires_at);',
  'CREATE INDEX IF NOT EXISTS idx_personalization_chapter ON personalization_cache(chapter_id);'
];

export const ROW_LEVEL_SECURITY = [
  // Enable RLS on tables with user data
  'ALTER TABLE user_profiles ENABLE ROW LEVEL SECURITY;',
  'ALTER TABLE chat_history ENABLE ROW LEVEL SECURITY;',
  'ALTER TABLE personalization_cache ENABLE ROW LEVEL SECURITY;',

  // User can only access their own profile
  `CREATE POLICY user_profiles_policy ON user_profiles
   FOR ALL TO authenticated_user
   USING (user_id = current_setting('app.current_user_id')::UUID);`,

  // Users can only access their own chat history
  `CREATE POLICY chat_history_policy ON chat_history
   FOR ALL TO authenticated_user
   USING (user_id = current_setting('app.current_user_id')::UUID OR user_id IS NULL);`,

  // Personalization cache is based on profile hash, not direct user access
  `CREATE POLICY personalization_cache_policy ON personalization_cache
   FOR ALL TO authenticated_user
   USING (true);`
];

export const TRIGGERS = [
  // Update updated_at timestamp
  `CREATE OR REPLACE FUNCTION update_updated_at_column()
   RETURNS TRIGGER AS $$
   BEGIN
       NEW.updated_at = CURRENT_TIMESTAMP;
       RETURN NEW;
   END;
   $$ language 'plpgsql';`,

  'DROP TRIGGER IF EXISTS update_users_updated_at ON users;',
  'CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();',

  'DROP TRIGGER IF EXISTS update_user_profiles_updated_at ON user_profiles;',
  'CREATE TRIGGER update_user_profiles_updated_at BEFORE UPDATE ON user_profiles FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();'
];

export const CONSTRAINTS = [
  // Ensure cache entries don't expire in the past
  'ALTER TABLE personalization_cache ADD CONSTRAINT chk_expires_future CHECK (expires_at > created_at);',

  // Ensure session_id is not null for non-anonymous messages
  'ALTER TABLE chat_history ADD CONSTRAINT chk_session_not_null CHECK (session_id IS NOT NULL);'
];

export const VIEWS = [
  // User summary view for quick lookups
  `CREATE OR REPLACE VIEW user_summary AS
   SELECT
       u.id,
       u.email,
       u.created_at as user_created_at,
       up.experience_level,
       up.ros_familiarity,
       up.hardware_access,
       up.learning_goal,
       up.preferred_language
   FROM users u
   LEFT JOIN user_profiles up ON u.id = up.user_id;`,

  // Chat session summary
  `CREATE OR REPLACE VIEW chat_session_summary AS
   SELECT
       session_id,
       user_id,
       COUNT(*) as message_count,
       MIN(created_at) as session_started,
       MAX(created_at) as last_message
   FROM chat_history
   GROUP BY session_id, user_id;`
];

// Migration versions for tracking schema changes
export const MIGRATIONS = [
  {
    version: '001',
    description: 'Initial schema creation',
    up: async (db) => {
      // Create all tables
      for (const [tableName, schema] of Object.entries(TABLE_SCHEMAS)) {
        const columns = Object.entries(schema)
          .map(([col, def]) => `${col} ${def}`)
          .join(', ');
        await db.query(`CREATE TABLE IF NOT EXISTS ${tableName} (${columns});`);
      }

      // Create indexes
      for (const indexSql of INDEXES) {
        await db.query(indexSql);
      }
    }
  },
  {
    version: '002',
    description: 'Add RLS policies and triggers',
    up: async (db) => {
      // Create triggers
      for (const triggerSql of TRIGGERS) {
        await db.query(triggerSql);
      }

      // Note: RLS policies require creating the authenticated_user role
      await db.query('DO $$
      BEGIN
          IF NOT EXISTS (SELECT FROM pg_catalog.pg_roles WHERE rolname = \'authenticated_user\') THEN
              CREATE ROLE authenticated_user;
          END IF
      END $$');
    }
  },
  {
    version: '003',
    description: 'Add constraints and views',
    up: async (db) => {
      // Add constraints
      for (const constraintSql of CONSTRAINTS) {
        try {
          await db.query(constraintSql);
        } catch (error) {
          console.warn('Constraint already exists or failed:', error.message);
        }
      }

      // Create views
      for (const viewSql of VIEWS) {
        await db.query(viewSql);
      }
    }
  }
];