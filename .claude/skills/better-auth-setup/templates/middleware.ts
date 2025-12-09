import { NextResponse } from "next/server";
import type { NextRequest } from "next/server";

// Define public routes that don't require authentication
const publicRoutes = [
  "/auth/signin",
  "/auth/signup",
  "/auth/forgot-password",
  "/auth/reset-password",
  "/auth/error",
  "/api/auth",
  "/api/auth/callback",
  "/api/auth/error",
  "/api/auth/verify-email",
];

// Define API routes
const apiRoutes = ["/api"];

export function middleware(request: NextRequest) {
  const { pathname } = request.nextUrl;

  // Check if it's an API route
  const isApiRoute = pathname.startsWith("/api");

  // Apply rate limiting to API routes
  if (isApiRoute && pathname.startsWith("/api/auth")) {
    const ip = request.ip ||
              request.headers.get("x-forwarded-for") ||
              request.headers.get("x-real-ip") ||
              "unknown";

    // Simple in-memory rate limiting
    // In production, use Redis or similar
    const limit = 10; // requests per minute
    const windowMs = 60 * 1000;
    const now = Date.now();

    // This is a basic implementation
    // Use a proper rate limiting solution in production
    const rateLimitKey = `rate_limit:${ip}`;
    const requests = parseInt(request.cookies.get(rateLimitKey)?.value || "0");
    const lastRequest = parseInt(request.cookies.get(`${rateLimitKey}_time`)?.value || "0");

    if (now - lastRequest > windowMs) {
      // Reset counter
      const response = NextResponse.next();
      response.cookies.set(rateLimitKey, "1", { httpOnly: true });
      response.cookies.set(`${rateLimitKey}_time`, now.toString(), { httpOnly: true });
      return response;
    }

    if (requests >= limit) {
      return new NextResponse(
        JSON.stringify({ error: "Too many requests" }),
        {
          status: 429,
          headers: {
            "Content-Type": "application/json",
            "Retry-After": "60",
          },
        }
      );
    }

    // Increment counter
    const response = NextResponse.next();
    response.cookies.set(rateLimitKey, (requests + 1).toString(), { httpOnly: true });
    return response;
  }

  // Add security headers
  const response = NextResponse.next();

  // Security headers
  response.headers.set("X-Content-Type-Options", "nosniff");
  response.headers.set("X-Frame-Options", "DENY");
  response.headers.set("X-XSS-Protection", "1; mode=block");
  response.headers.set("Referrer-Policy", "strict-origin-when-cross-origin");

  // HSTS (only in production)
  if (process.env.NODE_ENV === "production") {
    response.headers.set(
      "Strict-Transport-Security",
      "max-age=31536000; includeSubDomains; preload"
    );
  }

  // CSP Header (adjust as needed for your app)
  response.headers.set(
    "Content-Security-Policy",
    [
      "default-src 'self'",
      "script-src 'self' 'unsafe-inline' 'unsafe-eval'",
      "style-src 'self' 'unsafe-inline'",
      "img-src 'self' data: https: blob:",
      "font-src 'self' data:",
      "connect-src 'self'",
      "frame-ancestors 'none'",
      "base-uri 'self'",
      "form-action 'self'",
    ].join("; ")
  );

  // Handle authentication redirects
  const token = request.cookies.get("better-auth.session_token")?.value;
  const isPublicRoute = publicRoutes.some((route) => pathname.startsWith(route));

  // If user is not authenticated and trying to access protected route
  if (!token && !isPublicRoute && !pathname.startsWith("/_next")) {
    const loginUrl = new URL("/auth/signin", request.url);
    loginUrl.searchParams.set("callbackUrl", pathname);
    return NextResponse.redirect(loginUrl);
  }

  // If user is authenticated and trying to access auth routes
  if (token && pathname.startsWith("/auth")) {
    const callbackUrl = request.nextUrl.searchParams.get("callbackUrl");
    const redirectUrl = callbackUrl ? new URL(callbackUrl, request.url) : new URL("/dashboard", request.url);
    return NextResponse.redirect(redirectUrl);
  }

  return response;
}

export const config = {
  matcher: [
    /*
     * Match all request paths except for the ones starting with:
     * - _next/static (static files)
     * - _next/image (image optimization files)
     * - favicon.ico (favicon file)
     * - public folder
     */
    "/((?!_next/static|_next/image|favicon.ico|.*\\.(?:svg|png|jpg|jpeg|gif|webp)$).*)",
  ],
};