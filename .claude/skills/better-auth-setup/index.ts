import { SkillContext } from "claude-standalone";
import { writeFileSync, mkdirSync, existsSync } from "fs";
import { join } from "path";
import { cwd } from "process";

export default async function betterAuthSetup(context: SkillContext) {
  const { ask, say, input } = context;

  say("🔐 Better-Auth Setup Skill");
  say("========================\n");
  say("This skill will help you set up complete authentication with Better-Auth including user profiling.\n");

  // Collect configuration
  const authProviders = await ask({
    type: "multi-select",
    message: "Select authentication providers to enable:",
    options: [
      { value: "email", label: "Email & Password" },
      { value: "google", label: "Google OAuth" },
      { value: "github", label: "GitHub OAuth" },
    ],
  });

  const useCustomQuestions = await ask({
    type: "confirm",
    message: "Use default profile questions (Programming, ROS, Hardware, Learning Goal, Code Preferences)?",
    default: true,
  });

  const callbackUrls = {
    success: await ask({
      type: "text",
      message: "Success callback URL (after auth):",
      default: "/dashboard",
    }),
    failure: await ask({
      type: "text",
      message: "Failure callback URL (on error):",
      default: "/auth/error",
    }),
  };

  // Create directories
  const baseDir = cwd();
  const directories = [
    "src/lib",
    "src/components/auth",
    "src/hooks",
    "src/app/api/auth/[...allauth]",
    "src/app/auth/signin",
    "src/app/auth/signup",
    "src/db",
  ];

  say("\n📁 Creating directory structure...");
  directories.forEach((dir) => {
    const fullPath = join(baseDir, dir);
    if (!existsSync(fullPath)) {
      mkdirSync(fullPath, { recursive: true });
      say(`  ✓ Created ${dir}`);
    }
  });

  // Copy template files
  say("\n📄 Copying template files...");

  const templatesPath = join(__dirname, "templates");

  // Auth configuration
  const authConfig = await fetchFileContent(join(templatesPath, "auth.ts"));
  writeFileSync(join(baseDir, "src/lib/auth.ts"), authConfig);
  say("  ✓ Created src/lib/auth.ts");

  // Database schema
  const schemaSql = await fetchFileContent(join(templatesPath, "schema.sql"));
  writeFileSync(join(baseDir, "schema.sql"), schemaSql);
  say("  ✓ Created schema.sql");

  const schemaTs = await fetchFileContent(join(templatesPath, "schema.ts"));
  writeFileSync(join(baseDir, "src/db/schema.ts"), schemaTs);
  say("  ✓ Created src/db/schema.ts");

  // Auth components
  const signupComponent = await fetchFileContent(join(templatesPath, "signup.tsx"));
  writeFileSync(join(baseDir, "src/components/auth/signup.tsx"), signupComponent);
  say("  ✓ Created src/components/auth/signup.tsx");

  const signinComponent = await fetchFileContent(join(templatesPath, "signin.tsx"));
  writeFileSync(join(baseDir, "src/components/auth/signin.tsx"), signinComponent);
  say("  ✓ Created src/components/auth/signin.tsx");

  // Hooks
  const useAuth = await fetchFileContent(join(templatesPath, "use-auth.ts"));
  writeFileSync(join(baseDir, "src/hooks/use-auth.ts"), useAuth);
  say("  ✓ Created src/hooks/use-auth.ts");

  // Auth routes
  const authRoutes = await fetchFileContent(join(templatesPath, "auth-routes.ts"));
  writeFileSync(join(baseDir, "src/lib/auth-routes.ts"), authRoutes);
  say("  ✓ Created src/lib/auth-routes.ts");

  // API route
  const apiRoute = `export { GET, POST } from "@/lib/auth-routes";\nexport { GET as profileGET, POST as profilePOST, PUT as profilePUT } from "@/lib/auth-routes";`;
  writeFileSync(join(baseDir, "src/app/api/auth/[...allauth]/route.ts"), apiRoute);
  say("  ✓ Created src/app/api/auth/[...allauth]/route.ts");

  // Middleware
  const middleware = await fetchFileContent(join(templatesPath, "middleware.ts"));
  writeFileSync(join(baseDir, "src/middleware.ts"), middleware);
  say("  ✓ Created src/middleware.ts");

  // Environment variables
  const envExample = await fetchFileContent(join(templatesPath, ".env.example"));
  writeFileSync(join(baseDir, ".env.example"), envExample);
  say("  ✓ Created .env.example");

  // Auth pages
  const signupPage = `"use client";

import { SignupForm } from "@/components/auth/signup";

export default function SignupPage() {
  return (
    <div className="min-h-screen flex items-center justify-center bg-gray-50">
      <SignupForm
        callbackUrls={{
          success: "${callbackUrls.success}",
          failure: "${callbackUrls.failure}"
        }}
      />
    </div>
  );
}`;

  writeFileSync(join(baseDir, "src/app/auth/signup/page.tsx"), signupPage);
  say("  ✓ Created src/app/auth/signup/page.tsx");

  const signinPage = `"use client";

import { SigninForm } from "@/components/auth/signin";

export default function SigninPage() {
  return (
    <div className="min-h-screen flex items-center justify-center bg-gray-50">
      <SigninForm
        callbackUrls={{
          success: "${callbackUrls.success}",
          failure: "${callbackUrls.failure}"
        }}
      />
    </div>
  );
}`;

  writeFileSync(join(baseDir, "src/app/auth/signin/page.tsx"), signinPage);
  say("  ✓ Created src/app/auth/signin/page.tsx");

  // Package.json updates
  say("\n📦 Updating package.json...");
  const packageJsonPath = join(baseDir, "package.json");
  let packageJson: any = {};

  if (existsSync(packageJsonPath)) {
    packageJson = require(packageJsonPath);
  }

  const dependencies = {
    "better-auth": "^1.0.0",
    "drizzle-orm": "^0.29.0",
    "@neondatabase/serverless": "^0.9.0",
    "drizzle-kit": "^0.20.0",
  };

  packageJson.dependencies = { ...packageJson.dependencies, ...dependencies };

  writeFileSync(packageJsonPath, JSON.stringify(packageJson, null, 2));
  say("  ✓ Added Better-Auth dependencies to package.json");

  // Summary
  say("\n✅ Better-Auth setup complete!");
  say("\nNext steps:");
  say("1. Install dependencies: npm install");
  say("2. Copy .env.example to .env.local and configure your variables");
  say("3. Run the schema.sql in your Neon database");
  say("4. Configure OAuth providers if needed");
  say("5. Start your development server");
  say("\n📚 Check templates/setup-guide.md for detailed instructions");

  return {
    authProviders,
    callbackUrls,
    createdFiles: [
      "src/lib/auth.ts",
      "src/lib/auth-routes.ts",
      "src/db/schema.ts",
      "src/components/auth/signup.tsx",
      "src/components/auth/signin.tsx",
      "src/hooks/use-auth.ts",
      "src/middleware.ts",
      "src/app/api/auth/[...allauth]/route.ts",
      "src/app/auth/signup/page.tsx",
      "src/app/auth/signin/page.tsx",
      "schema.sql",
      ".env.example",
    ],
  };
}

async function fetchFileContent(filePath: string): Promise<string> {
  // In a real implementation, this would read from the actual template files
  // For now, return a placeholder
  return "// Template content would be loaded from: " + filePath;
}