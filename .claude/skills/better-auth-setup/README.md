# Better-Auth Setup Skill

Complete authentication setup using Better-Auth with user profiling capabilities.

## Features

- **Multiple Auth Providers**: Email, Google, GitHub, and more
- **User Profiling**: Collect custom information during signup
- **Security**: CSRF protection, rate limiting, secure cookies
- **Database Integration**: Neon PostgreSQL schema ready
- **React Components**: Pre-built signup/signin forms
- **Session Management**: Custom hooks for auth state

## Usage

```bash
/skill better-auth-setup
```

The skill will prompt you for:
- Authentication providers (email, google, github)
- Profile questions to collect
- Callback URLs for redirects

## Default Profile Questions

1. Programming experience level (Beginner/Intermediate/Advanced)
2. Familiarity with ROS (None/Basic/Proficient)
3. Hardware access (Simulation Only/Jetson Kit/Full Robot Lab)
4. Primary learning goal (Career/Research/Hobby)
5. Preferred code examples (Python/C++/Both)

## Output Structure

```
src/
├── lib/
│   └── auth.ts          # Better-Auth configuration
├── components/
│   └── auth/
│       ├── signup.tsx   # Signup component with profile
│       └── signin.tsx   # Signin component
├── hooks/
│   └── use-auth.ts      # Session management hooks
└── db/
    └── schema.ts        # Database schema
```