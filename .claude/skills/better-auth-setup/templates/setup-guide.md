# Better-Auth Setup Guide

This guide will help you set up Better-Auth authentication with user profiling in your Next.js application.

## Prerequisites

- Next.js 14+ with App Router
- Neon PostgreSQL database
- Node.js 18+

## Installation

1. Install required dependencies:

```bash
npm install better-auth better-auth/adapters/neon drizzle-orm drizzle-kit @types/react @types/react-dom
```

## Configuration

### 1. Environment Variables

Copy `.env.example` to `.env.local` and configure:

```env
# Generate a secure secret
openssl rand -base64 32

# Add to .env.local
BETTER_AUTH_SECRET="your-secret-here"
DATABASE_URL="your-neon-database-url"
BETTER_AUTH_URL="http://localhost:3000"
```

### 2. Database Setup

Run the schema in your Neon database:

```bash
# Copy the schema file
cp templates/schema.sql schema.sql

# Run in Neon SQL editor or connect with psql
```

### 3. Auth Configuration

1. Copy `templates/auth.ts` to `src/lib/auth.ts`

2. Update the configuration based on your needs:
   - Enable/disable auth providers
   - Adjust session settings
   - Configure rate limiting

### 4. API Routes

Create `src/app/api/auth/[...allauth]/route.ts`:

```typescript
export { GET, POST } from "@/lib/auth-routes";
```

### 5. Middleware

Copy `templates/middleware.ts` to `src/middleware.ts`

### 6. Auth Components

1. Create `src/components/auth/` directory
2. Copy `templates/signup.tsx` and `templates/signin.tsx`
3. Create pages:

```typescript
// src/app/auth/signup/page.tsx
import { SignupForm } from "@/components/auth/signup";
import { authConfig } from "@/lib/auth";

export default function SignupPage() {
  return (
    <SignupForm
      callbackUrls={{
        success: "/dashboard",
        failure: "/auth/error"
      }}
    />
  );
}
```

### 7. Session Management

Copy `templates/use-auth.ts` to `src/hooks/use-auth.ts`

### 8. Social Provider Setup

For Google OAuth:
1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select existing
3. Enable Google+ API
4. Create OAuth 2.0 credentials
5. Add redirect URI: `http://localhost:3000/api/auth/callback/google`

For GitHub OAuth:
1. Go to GitHub Settings > Developer settings > OAuth Apps
2. Create new OAuth App
3. Set callback URL: `http://localhost:3000/api/auth/callback/github`

## Usage Examples

### Protected Routes

```typescript
// src/app/dashboard/page.tsx
import { useRequireAuth } from "@/hooks/use-auth";

export default function Dashboard() {
  const { user, isAuthenticated } = useRequireAuth();

  if (!isAuthenticated) {
    return <div>Loading...</div>;
  }

  return <div>Welcome, {user?.name}!</div>;
}
```

### Profile Management

```typescript
// src/app/profile/page.tsx
import { useAuth } from "@/hooks/use-auth";

export default function Profile() {
  const { user, profile, updateProfile } = useAuth();

  const handleUpdate = async (data) => {
    const success = await updateProfile(data);
    if (success) {
      alert("Profile updated!");
    }
  };

  return (
    <div>
      <h1>Profile</h1>
      <p>Programming Experience: {profile?.programmingExperience}</p>
      {/* Form for updating profile */}
    </div>
  );
}
```

### Checking Profile Completion

```typescript
// src/app/app-layout.tsx
import { useRequireProfile } from "@/hooks/use-auth";

export default function AppLayout({ children }) {
  const { hasCompleteProfile } = useRequireProfile();

  if (!hasCompleteProfile) {
    return <div>Please complete your profile first</div>;
  }

  return <>{children}</>;
}
```

## Security Features

The implementation includes:

1. **CSRF Protection**: Prevents cross-site request forgery
2. **Rate Limiting**: Limits authentication attempts
3. **Secure Cookies**: HttpOnly, SameSite, and secure flags
4. **Password Requirements**: Minimum 8 characters
5. **Session Management**: Automatic session refresh and expiration
6. **Security Headers**: XSS protection, content type options, etc.

## Database Schema

The user profile includes:
- Programming experience level
- ROS familiarity
- Hardware access type
- Primary learning goal
- Preferred code examples

## Customization

### Adding New Profile Questions

1. Update the database schema (`schema.sql`)
2. Modify the TypeScript types in `schema.ts`
3. Update the signup form component
4. Adjust the profile interface as needed

### Custom Auth Providers

Add new providers in `auth.ts`:

```typescript
socialProviders: {
  discord: {
    clientId: process.env.DISCORD_CLIENT_ID!,
    clientSecret: process.env.DISCORD_CLIENT_SECRET!,
    enabled: true,
  },
}
```

### Custom Rate Limiting

Adjust rate limits in `middleware.ts` and `auth-routes.ts` based on your needs.

## Production Considerations

1. **Environment Variables**: Ensure all secrets are properly configured
2. **Database**: Use a production Neon instance
3. **HTTPS**: Always use HTTPS in production
4. **Domain Whitelist**: Update `BETTER_AUTH_TRUSTED_ORIGINS`
5. **Monitoring**: Set up logging for auth events
6. **Session Storage**: Consider Redis for session storage in production

## Troubleshooting

### Common Issues

1. **CSRF Errors**: Check CORS and origin configuration
2. **Rate Limiting**: Adjust limits or implement distributed rate limiting
3. **Database Connection**: Verify DATABASE_URL format
4. **Social Auth**: Ensure redirect URIs match exactly

### Debug Mode

Enable debug logging:

```typescript
export const auth = betterAuth({
  ...authConfig,
  advanced: {
    ...authConfig.advanced,
    debug: process.env.NODE_ENV === "development",
  },
});
```