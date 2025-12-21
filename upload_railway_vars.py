#!/usr/bin/env python3
"""
Railway Variables Upload Script
Uploads environment variables from .env.railway to Railway project
Requires: RAILWAY_TOKEN environment variable
"""

import os
import sys
import json
import requests
from pathlib import Path

def load_env_file(filepath):
    """Load environment variables from .env file"""
    variables = {}
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            # Skip comments and empty lines
            if not line or line.startswith('#'):
                continue
            # Parse KEY=VALUE
            if '=' in line:
                key, value = line.split('=', 1)
                key = key.strip()
                value = value.strip()
                # Remove quotes if present
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                variables[key] = value
    return variables

def upload_to_railway(variables, project_id, environment_id, token):
    """Upload variables to Railway using GraphQL API"""
    url = "https://backboard.railway.app/graphql/v2"
    headers = {
        "Authorization": f"Bearer {token}",
        "Content-Type": "application/json"
    }

    results = []
    for key, value in variables.items():
        # GraphQL mutation to set variable
        mutation = """
        mutation VariableUpsert($input: VariableUpsertInput!) {
          variableUpsert(input: $input)
        }
        """

        payload = {
            "query": mutation,
            "variables": {
                "input": {
                    "projectId": project_id,
                    "environmentId": environment_id,
                    "name": key,
                    "value": value
                }
            }
        }

        try:
            response = requests.post(url, headers=headers, json=payload)
            response.raise_for_status()
            results.append((key, "✅"))
        except Exception as e:
            results.append((key, f"❌ {str(e)}"))

    return results

def main():
    """Main execution"""
    print("=" * 80)
    print("RAILWAY VARIABLES UPLOAD SCRIPT")
    print("=" * 80)

    # Check for Railway token
    token = os.getenv('RAILWAY_TOKEN')
    if not token:
        print("\n❌ Error: RAILWAY_TOKEN environment variable not set")
        print("\nTo get your token:")
        print("1. Go to: https://railway.app/account/tokens")
        print("2. Create a new token")
        print("3. Run: export RAILWAY_TOKEN='your-token-here'")
        print("4. Run this script again")
        sys.exit(1)

    # Load variables from .env.railway
    env_file = Path('.env.railway')
    if not env_file.exists():
        print(f"\n❌ Error: {env_file} not found")
        sys.exit(1)

    print(f"\n📄 Loading variables from: {env_file}")
    variables = load_env_file(env_file)
    print(f"✅ Loaded {len(variables)} variables")

    # Get project and environment IDs
    print("\n📋 Please provide your Railway project details:")
    print("   (Find these in your Railway dashboard URL)")
    print("   Example: https://railway.app/project/abc123/service/xyz789")

    project_id = input("\nProject ID: ").strip()
    environment_id = input("Environment ID (or press Enter for production): ").strip()

    if not environment_id:
        environment_id = "production"

    # Upload variables
    print(f"\n🚀 Uploading {len(variables)} variables to Railway...")
    print("-" * 80)

    results = upload_to_railway(variables, project_id, environment_id, token)

    # Display results
    print("\nResults:")
    success_count = 0
    for key, status in results:
        print(f"  {status} {key}")
        if "✅" in status:
            success_count += 1

    print("-" * 80)
    print(f"\n✅ Successfully uploaded {success_count}/{len(variables)} variables")

    if success_count < len(variables):
        print(f"❌ Failed to upload {len(variables) - success_count} variables")
        sys.exit(1)

    print("\n✅ All variables uploaded successfully!")
    print("=" * 80)

if __name__ == "__main__":
    main()
