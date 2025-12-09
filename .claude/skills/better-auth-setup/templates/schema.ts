import { pgTable, uuid, varchar, timestamp } from "drizzle-orm/pg-core";
import { relations } from "drizzle-orm";

// This assumes you're using Drizzle ORM with the Better-Auth Neon adapter
// The auth.users table is created by Better-Auth

export const userProfiles = pgTable("user_profiles", {
  id: uuid("id").primaryKey().defaultRandom(),
  userId: uuid("user_id")
    .notNull()
    .references(() => authUsers.id, { onDelete: "cascade" }),
  programmingExperience: varchar("programming_experience", {
    enum: ["Beginner", "Intermediate", "Advanced"]
  }).notNull(),
  rosFamiliarity: varchar("ros_familiarity", {
    enum: ["None", "Basic", "Proficient"]
  }).notNull(),
  hardwareAccess: varchar("hardware_access", {
    enum: ["Simulation Only", "Jetson Kit", "Full Robot Lab"]
  }).notNull(),
  learningGoal: varchar("learning_goal", {
    enum: ["Career", "Research", "Hobby"]
  }).notNull(),
  preferredCode: varchar("preferred_code", {
    enum: ["Python", "C++", "Both"]
  }).notNull(),
  createdAt: timestamp("created_at").defaultNow(),
  updatedAt: timestamp("updated_at").defaultNow(),
});

// This references the Better-Auth users table
export const authUsers = pgTable("auth_users", {
  id: uuid("id").primaryKey(),
  email: varchar("email").notNull().unique(),
  name: varchar("name"),
  image: varchar("image"),
  // Other fields managed by Better-Auth
});

export const userProfilesRelations = relations(userProfiles, ({ one }) => ({
  user: one(authUsers, {
    fields: [userProfiles.userId],
    references: [authUsers.id],
  }),
}));

export type UserProfile = typeof userProfiles.$inferSelect;
export type NewUserProfile = typeof userProfiles.$inferInsert;