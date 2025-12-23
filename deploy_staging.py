#!/usr/bin/env python3
"""
Automated Railway Staging Deployment Script
Generates Railway CLI commands from .env.staging
"""
import os
import re
from pathlib import Path

def parse_env_file(env_file_path):
    """Parse .env file and return key-value pairs"""
    env_vars = {}

    with open(env_file_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()

            # Skip comments and empty lines
            if not line or line.startswith('#'):
                continue

            # Parse KEY=VALUE
            match = re.match(r'^([A-Z_][A-Z0-9_]*)=(.*)$', line)
            if match:
                key = match.group(1)
                value = match.group(2)

                # Remove quotes if present
                value = value.strip('"').strip("'")

                env_vars[key] = value

    return env_vars

def generate_railway_commands(env_vars):
    """Generate Railway CLI commands to set environment variables"""
    commands = []

    # Critical variables that MUST be set
    critical_vars = [
        'DATABASE_URL',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'OPENAI_API_KEY',
        'ENVIRONMENT',
        'CORS_ORIGINS',
        'FRONTEND_URL',
    ]

    # Variables to skip (Railway sets these automatically)
    skip_vars = ['API_PORT', 'API_HOST', 'RAILWAY_ENVIRONMENT', 'RAILWAY_GIT_BRANCH']

    print("=" * 80)
    print("RAILWAY STAGING DEPLOYMENT - ENVIRONMENT VARIABLES")
    print("=" * 80)
    print()

    # Generate commands
    for key, value in sorted(env_vars.items()):
        if key in skip_vars:
            print(f"[SKIP] {key} (Railway sets automatically)")
            continue

        is_critical = key in critical_vars
        prefix = "[CRITICAL]" if is_critical else "[OPTIONAL]"

        # Escape quotes in value
        escaped_value = value.replace('"', '\\"')

        # Generate Railway CLI command
        cmd = f'railway variables set {key}="{escaped_value}"'
        commands.append(cmd)

        # Show placeholder detection
        if 'your-' in value.lower() or 'placeholder' in value.lower():
            print(f"{prefix} {key} = [PLACEHOLDER - NEEDS UPDATE]")
        else:
            # Show first 50 chars for security
            display_value = value[:50] + "..." if len(value) > 50 else value
            print(f"{prefix} {key} = {display_value}")

    return commands

def main():
    # Path to .env.staging
    env_staging_path = Path(__file__).parent / 'backend' / '.env.staging'

    if not env_staging_path.exists():
        print(f"[X] Error: .env.staging not found at {env_staging_path}")
        return

    print(f"[*] Reading environment variables from: {env_staging_path}")
    print()

    # Parse .env.staging
    env_vars = parse_env_file(env_staging_path)

    # Generate Railway commands
    commands = generate_railway_commands(env_vars)

    # Write deployment script
    deploy_script_path = Path(__file__).parent / 'deploy_to_railway_staging.sh'

    with open(deploy_script_path, 'w', encoding='utf-8') as f:
        f.write("#!/bin/bash\n")
        f.write("# =============================================================================\n")
        f.write("# Railway Staging Deployment Script - Auto-generated\n")
        f.write("# =============================================================================\n")
        f.write("# Usage: bash deploy_to_railway_staging.sh\n")
        f.write("# Prerequisites: Railway CLI installed and authenticated\n")
        f.write("# =============================================================================\n\n")

        f.write("set -e  # Exit on error\n\n")

        f.write("echo '============================================================'\n")
        f.write("echo 'Railway Staging Deployment - Phase 4B'\n")
        f.write("echo '============================================================'\n\n")

        f.write("# Step 1: Link to Railway project\n")
        f.write("echo '[*] Linking to Railway project...'\n")
        f.write("railway link\n\n")

        f.write("# Step 2: Switch to staging environment\n")
        f.write("echo '[*] Switching to staging environment...'\n")
        f.write("railway environment staging\n\n")

        f.write("# Step 3: Set environment variables\n")
        f.write("echo '[*] Setting environment variables...'\n")
        for cmd in commands:
            f.write(f"{cmd}\n")

        f.write("\n# Step 4: Deploy to Railway\n")
        f.write("echo '[*] Deploying to Railway staging...'\n")
        f.write("cd backend\n")
        f.write("railway up\n\n")

        f.write("echo '============================================================'\n")
        f.write("echo '[OK] Deployment Complete!'\n")
        f.write("echo '============================================================'\n")
        f.write("echo 'Next steps:'\n")
        f.write("echo '  1. Check deployment logs: railway logs'\n")
        f.write("echo '  2. Get deployment URL: railway status'\n")
        f.write("echo '  3. Test health endpoint: curl https://your-app.railway.app/health'\n")

    print()
    print("=" * 80)
    print("[OK] Deployment script generated!")
    print("=" * 80)
    print()
    print(f"[*] Script location: {deploy_script_path}")
    print()
    print("=" * 80)
    print("MANUAL STEPS REQUIRED:")
    print("=" * 80)
    print()
    print("1. Authenticate with Railway CLI:")
    print("   railway login")
    print()
    print("2. Create staging environment in Railway dashboard:")
    print("   https://railway.app/dashboard")
    print("   - Select your project")
    print("   - Create environment: 'staging'")
    print("   - Set source branch: '1-personalization-engine'")
    print()
    print("3. Run the deployment script:")
    print(f"   bash {deploy_script_path}")
    print()
    print("=" * 80)
    print()
    print("[!] IMPORTANT: Update placeholders in .env.staging before deploying:")
    print("    - JWT_PRIVATE_KEY and JWT_PUBLIC_KEY (generate with OpenSSL)")
    print("    - CSRF_SECRET_KEY (generate with Python secrets)")
    print("    - GOOGLE_CLIENT_ID and GOOGLE_CLIENT_SECRET (create OAuth app)")
    print("    - GITHUB_CLIENT_ID and GITHUB_CLIENT_SECRET (create OAuth app)")
    print()

if __name__ == "__main__":
    main()
