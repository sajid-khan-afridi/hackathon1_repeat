import { betterAuth } from "better-auth";
import { neonAdapter } from "better-auth/adapters/neon";
import { admin, openAPI } from "better-auth/plugins";

export interface UserProfile {
  id: string;
  userId: string;
  programmingExperience: "Beginner" | "Intermediate" | "Advanced";
  rosFamiliarity: "None" | "Basic" | "Proficient";
  hardwareAccess: "Simulation Only" | "Jetson Kit" | "Full Robot Lab";
  learningGoal: "Career" | "Research" | "Hobby";
  preferredCode: "Python" | "C++" | "Both";
  createdAt: Date;
  updatedAt: Date;
}

export const authConfig = {
  database: {
    provider: "neon",
    url: process.env.DATABASE_URL,
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: true,
    minPasswordLength: 8,
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID!,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET!,
      enabled: true,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
      enabled: true,
    },
  },
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    },
  },
  rateLimit: {
    window: 60, // 1 minute
    max: 10, // 10 attempts per window
  },
  account: {
    accountLinking: {
      enabled: true,
      trustedProviders: ["google", "github"],
    },
  },
  advanced: {
    generateId: false,
    crossSubDomainCookies: {
      enabled: false,
    },
  },
  plugins: [admin(), openAPI()],
};

export const auth = betterAuth(authConfig);