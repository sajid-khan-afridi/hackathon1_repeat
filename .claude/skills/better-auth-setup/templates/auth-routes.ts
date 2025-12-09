import { auth } from "@/lib/auth";
import { toNextJsHandler } from "better-auth/next-js";
import { NextRequest } from "next/server";

// Rate limiting configuration
const rateLimitMap = new Map();

function rateLimit(
  identifier: string,
  limit: number = 10,
  windowMs: number = 60 * 1000
) {
  const now = Date.now();
  const windowStart = now - windowMs;

  // Clean old entries
  for (const [key, timestamps] of rateLimitMap.entries()) {
    const validTimestamps = (timestamps as number[]).filter(t => t > windowStart);
    if (validTimestamps.length === 0) {
      rateLimitMap.delete(key);
    } else {
      rateLimitMap.set(key, validTimestamps);
    }
  }

  // Check current identifier
  const timestamps = rateLimitMap.get(identifier) || [];
  const recentTimestamps = timestamps.filter((t: number) => t > windowStart);

  if (recentTimestamps.length >= limit) {
    return false;
  }

  // Add current timestamp
  rateLimitMap.set(identifier, [...recentTimestamps, now]);
  return true;
}

// Enhanced auth handler with security middleware
export const { GET, POST } = toNextJsHandler(auth, {
  // CSRF protection
  csrfMiddleware: (req: NextRequest) => {
    // Skip CSRF for GET requests and social auth callbacks
    if (req.method === "GET") return;

    const origin = req.headers.get("origin");
    const referer = req.headers.get("referer");
    const host = req.headers.get("host");

    // Allow same-origin requests
    if (origin && new URL(origin).host === host) return;
    if (referer && new URL(referer).host === host) return;

    // Allow specific auth endpoints
    const url = new URL(req.url);
    if (url.pathname.includes("/callback") ||
        url.pathname.includes("/error") ||
        url.pathname.includes("/verify-email")) {
      return;
    }

    // Block cross-origin requests for sensitive operations
    throw new Error("CSRF: Invalid origin");
  },

  // Rate limiting middleware
  rateLimitMiddleware: (req: NextRequest) => {
    const ip = req.headers.get("x-forwarded-for") ||
              req.headers.get("x-real-ip") ||
              "unknown";

    // Different limits for different endpoints
    const url = new URL(req.url);
    let limit = 10;

    if (url.pathname.includes("/sign-in")) {
      limit = 5; // Stricter for sign-in
    } else if (url.pathname.includes("/sign-up")) {
      limit = 3; // Even stricter for sign-up
    } else if (url.pathname.includes("/forgot-password")) {
      limit = 2; // Very strict for password reset
    }

    if (!rateLimit(ip, limit, 60 * 1000)) {
      throw new Error("Rate limit exceeded. Please try again later.");
    }
  },

  // Security headers middleware
  securityMiddleware: (response: Response) => {
    // Set security headers
    response.headers.set("X-Content-Type-Options", "nosniff");
    response.headers.set("X-Frame-Options", "DENY");
    response.headers.set("X-XSS-Protection", "1; mode=block");
    response.headers.set("Referrer-Policy", "strict-origin-when-cross-origin");

    // Content Security Policy (adjust as needed)
    response.headers.set(
      "Content-Security-Policy",
      "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; font-src 'self' data:"
    );
  },
});

// Profile API endpoint for user profile management
export async function POST(req: Request) {
  if (!auth) {
    return new Response("Auth not initialized", { status: 500 });
  }

  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session?.user) {
    return new Response("Unauthorized", { status: 401 });
  }

  const body = await req.json();

  try {
    // Save profile to database (implementation depends on your DB setup)
    const profile = await saveUserProfile(session.user.id, body);
    return Response.json(profile);
  } catch (error) {
    console.error("Error saving profile:", error);
    return new Response("Failed to save profile", { status: 500 });
  }
}

export async function PUT(req: Request) {
  if (!auth) {
    return new Response("Auth not initialized", { status: 500 });
  }

  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session?.user) {
    return new Response("Unauthorized", { status: 401 });
  }

  const body = await req.json();

  try {
    // Update profile in database
    const profile = await updateUserProfile(session.user.id, body);
    return Response.json(profile);
  } catch (error) {
    console.error("Error updating profile:", error);
    return new Response("Failed to update profile", { status: 500 });
  }
}

export async function GET(req: Request) {
  if (!auth) {
    return new Response("Auth not initialized", { status: 500 });
  }

  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session?.user) {
    return new Response("Unauthorized", { status: 401 });
  }

  try {
    // Fetch user profile from database
    const profile = await getUserProfile(session.user.id);
    return Response.json(profile);
  } catch (error) {
    console.error("Error fetching profile:", error);
    return new Response("Failed to fetch profile", { status: 500 });
  }
}

// Helper functions (implement based on your database setup)
async function saveUserProfile(userId: string, data: any) {
  // Implementation depends on your ORM/database
  // Example using Drizzle ORM:
  // const db = getDatabaseConnection();
  // const [profile] = await db.insert(userProfiles).values({
  //   userId,
  //   ...data,
  // }).returning();
  // return profile;
  throw new Error("Not implemented");
}

async function updateUserProfile(userId: string, data: any) {
  // Implementation depends on your ORM/database
  throw new Error("Not implemented");
}

async function getUserProfile(userId: string) {
  // Implementation depends on your ORM/database
  throw new Error("Not implemented");
}