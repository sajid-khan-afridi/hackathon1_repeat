#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Production Configuration Validation Script
Feature: 001-user-auth
Date: 2025-12-20

This script validates all required production environment variables
and security configurations before deployment.

Usage:
    python backend/validate_production_config.py
"""

import sys
import re
from pathlib import Path

# Set UTF-8 encoding for Windows console
if sys.platform == "win32":
    import codecs
    sys.stdout = codecs.getwriter("utf-8")(sys.stdout.detach())
    sys.stderr = codecs.getwriter("utf-8")(sys.stderr.detach())

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from backend.app.config import settings


class ProductionValidator:
    """Validate production configuration and security settings."""

    def __init__(self):
        self.errors = []
        self.warnings = []
        self.checks_passed = 0
        self.checks_total = 0

    def check(self, condition: bool, message: str, critical: bool = True) -> None:
        """
        Record a validation check result.

        Args:
            condition: True if check passed, False if failed
            message: Description of the check
            critical: If False, record as warning instead of error
        """
        self.checks_total += 1
        if condition:
            self.checks_passed += 1
            print(f"✅ {message}")
        else:
            if critical:
                self.errors.append(message)
                print(f"❌ {message}")
            else:
                self.warnings.append(message)
                print(f"⚠️  {message}")

    def validate_environment(self) -> None:
        """Validate environment configuration."""
        print("\n" + "="*80)
        print("🌍 Environment Configuration")
        print("="*80 + "\n")

        self.check(
            settings.is_production,
            f"ENVIRONMENT is set to 'production' (currently: '{settings.environment}')",
            critical=True
        )

        self.check(
            settings.log_format == "json",
            f"LOG_FORMAT is 'json' for structured logging (currently: '{getattr(settings, 'log_format', 'not set')}')",
            critical=False
        )

        self.check(
            settings.api_host == "0.0.0.0",
            f"API_HOST is '0.0.0.0' (currently: '{settings.api_host}')",
            critical=False
        )

    def validate_jwt_configuration(self) -> None:
        """Validate JWT token configuration."""
        print("\n" + "="*80)
        print("🔐 JWT Configuration")
        print("="*80 + "\n")

        self.check(
            settings.jwt_configured,
            "JWT keys are configured (JWT_PRIVATE_KEY and JWT_PUBLIC_KEY set)",
            critical=True
        )

        if settings.jwt_private_key:
            self.check(
                "-----BEGIN PRIVATE KEY-----" in settings.jwt_private_key or
                "-----BEGIN RSA PRIVATE KEY-----" in settings.jwt_private_key,
                "JWT_PRIVATE_KEY has valid PEM format",
                critical=True
            )

            self.check(
                "\\n" in settings.jwt_private_key,
                "JWT_PRIVATE_KEY is in single-line format with \\n separators",
                critical=True
            )

            # Check key length (approximate)
            key_length = len(settings.jwt_private_key.replace("\\n", "").replace(" ", ""))
            self.check(
                key_length > 1600,  # 2048-bit key should be ~1700+ characters
                f"JWT_PRIVATE_KEY appears to be 2048-bit or higher (length: {key_length})",
                critical=False
            )

        if settings.jwt_public_key:
            self.check(
                "-----BEGIN PUBLIC KEY-----" in settings.jwt_public_key,
                "JWT_PUBLIC_KEY has valid PEM format",
                critical=True
            )

        self.check(
            settings.jwt_algorithm == "RS256",
            f"JWT_ALGORITHM is RS256 (currently: '{settings.jwt_algorithm}')",
            critical=True
        )

        self.check(
            settings.jwt_access_token_expire_minutes <= 1440,
            f"JWT_ACCESS_TOKEN_EXPIRE_MINUTES is reasonable (currently: {settings.jwt_access_token_expire_minutes} minutes)",
            critical=False
        )

        self.check(
            settings.jwt_refresh_token_expire_days <= 90,
            f"JWT_REFRESH_TOKEN_EXPIRE_DAYS is reasonable (currently: {settings.jwt_refresh_token_expire_days} days)",
            critical=False
        )

    def validate_oauth_configuration(self) -> None:
        """Validate OAuth configuration."""
        print("\n" + "="*80)
        print("🔑 OAuth Configuration")
        print("="*80 + "\n")

        # GitHub OAuth
        self.check(
            settings.github_oauth_configured,
            "GitHub OAuth is configured (GITHUB_CLIENT_ID, CLIENT_SECRET, REDIRECT_URI set)",
            critical=True
        )

        if settings.github_redirect_uri:
            self.check(
                settings.github_redirect_uri.startswith("https://"),
                f"GITHUB_REDIRECT_URI uses HTTPS (currently: '{settings.github_redirect_uri}')",
                critical=True
            )

            self.check(
                "localhost" not in settings.github_redirect_uri.lower(),
                "GITHUB_REDIRECT_URI does not contain 'localhost'",
                critical=True
            )

        # Google OAuth (optional)
        if settings.google_oauth_configured:
            print("ℹ️  Google OAuth is configured (optional)")
            if settings.google_redirect_uri:
                self.check(
                    settings.google_redirect_uri.startswith("https://"),
                    f"GOOGLE_REDIRECT_URI uses HTTPS (currently: '{settings.google_redirect_uri}')",
                    critical=False
                )

    def validate_cors_configuration(self) -> None:
        """Validate CORS configuration."""
        print("\n" + "="*80)
        print("🌐 CORS Configuration")
        print("="*80 + "\n")

        cors_origins = settings.cors_origins_list

        self.check(
            len(cors_origins) > 0,
            f"CORS_ORIGINS is configured ({len(cors_origins)} origins)",
            critical=True
        )

        # Check for wildcard
        self.check(
            "*" not in settings.cors_origins,
            "CORS_ORIGINS does not use wildcard '*'",
            critical=True
        )

        # Check for localhost
        has_localhost = any("localhost" in origin.lower() for origin in cors_origins)
        self.check(
            not has_localhost,
            "CORS_ORIGINS does not include 'localhost'",
            critical=True
        )

        # Check for HTTP in production
        has_http = any(origin.startswith("http://") for origin in cors_origins)
        self.check(
            not has_http,
            "CORS_ORIGINS uses HTTPS only (no http:// origins)",
            critical=True
        )

        # Check for specific domains
        print(f"\nℹ️  Configured CORS origins:")
        for origin in cors_origins:
            print(f"   - {origin}")

    def validate_security_configuration(self) -> None:
        """Validate security settings."""
        print("\n" + "="*80)
        print("🛡️  Security Configuration")
        print("="*80 + "\n")

        self.check(
            settings.csrf_secret_key is not None and len(settings.csrf_secret_key) >= 32,
            f"CSRF_SECRET_KEY is set and strong (length: {len(settings.csrf_secret_key or '')})",
            critical=True
        )

        if settings.frontend_url:
            self.check(
                settings.frontend_url.startswith("https://"),
                f"FRONTEND_URL uses HTTPS (currently: '{settings.frontend_url}')",
                critical=True
            )

            self.check(
                "localhost" not in settings.frontend_url.lower(),
                "FRONTEND_URL does not contain 'localhost'",
                critical=True
            )

    def validate_database_configuration(self) -> None:
        """Validate database configuration."""
        print("\n" + "="*80)
        print("💾 Database Configuration")
        print("="*80 + "\n")

        self.check(
            settings.database_url is not None,
            "DATABASE_URL is configured",
            critical=True
        )

        if settings.database_url:
            self.check(
                "sslmode=require" in settings.database_url,
                "DATABASE_URL requires SSL (sslmode=require)",
                critical=True
            )

            # Hide password in output
            safe_db_url = re.sub(r':(.*?)@', ':***@', settings.database_url)
            print(f"ℹ️  Database URL: {safe_db_url}")

    def validate_rate_limiting(self) -> None:
        """Validate rate limiting configuration."""
        print("\n" + "="*80)
        print("⏱️  Rate Limiting")
        print("="*80 + "\n")

        self.check(
            settings.rate_limit_anonymous > 0,
            f"RATE_LIMIT_ANONYMOUS is configured (currently: {settings.rate_limit_anonymous}/hour)",
            critical=False
        )

        self.check(
            settings.rate_limit_authenticated > settings.rate_limit_anonymous,
            f"RATE_LIMIT_AUTHENTICATED > RATE_LIMIT_ANONYMOUS ({settings.rate_limit_authenticated} > {settings.rate_limit_anonymous})",
            critical=False
        )

        self.check(
            settings.rate_limit_anonymous <= 100,
            f"RATE_LIMIT_ANONYMOUS is reasonable (currently: {settings.rate_limit_anonymous}/hour)",
            critical=False
        )

    def validate_external_services(self) -> None:
        """Validate external service configurations."""
        print("\n" + "="*80)
        print("🌩️  External Services")
        print("="*80 + "\n")

        # OpenAI
        self.check(
            settings.openai_api_key is not None and settings.openai_api_key.startswith("sk-"),
            "OPENAI_API_KEY is configured and valid format",
            critical=True
        )

        # Qdrant
        self.check(
            settings.qdrant_url is not None,
            "QDRANT_URL is configured",
            critical=True
        )

        if settings.qdrant_url:
            self.check(
                settings.qdrant_url.startswith("https://"),
                f"QDRANT_URL uses HTTPS (currently: '{settings.qdrant_url}')",
                critical=True
            )

        self.check(
            settings.qdrant_api_key is not None,
            "QDRANT_API_KEY is configured",
            critical=True
        )

        self.check(
            settings.qdrant_collection is not None,
            f"QDRANT_COLLECTION is configured (currently: '{settings.qdrant_collection}')",
            critical=True
        )

    def print_summary(self) -> bool:
        """
        Print validation summary.

        Returns:
            True if all critical checks passed, False otherwise
        """
        print("\n" + "="*80)
        print("📊 Validation Summary")
        print("="*80 + "\n")

        print(f"Total checks: {self.checks_total}")
        print(f"Passed: {self.checks_passed}")
        print(f"Failed: {len(self.errors)}")
        print(f"Warnings: {len(self.warnings)}")

        if self.errors:
            print("\n❌ CRITICAL ERRORS (must fix before production deployment):")
            for i, error in enumerate(self.errors, 1):
                print(f"   {i}. {error}")

        if self.warnings:
            print("\n⚠️  WARNINGS (recommended to fix):")
            for i, warning in enumerate(self.warnings, 1):
                print(f"   {i}. {warning}")

        if not self.errors and not self.warnings:
            print("\n✅ ALL CHECKS PASSED - Configuration is ready for production!")
            return True
        elif not self.errors:
            print("\n⚠️  Configuration has warnings but is acceptable for production")
            print("   Consider addressing warnings before deployment")
            return True
        else:
            print("\n❌ VALIDATION FAILED - Do NOT deploy to production!")
            print("   Fix all critical errors before deploying")
            return False

    def run(self) -> int:
        """
        Run all validations.

        Returns:
            Exit code (0 for success, 1 for failure)
        """
        print("🚀 Production Configuration Validation")
        print(f"Environment: {settings.environment}\n")

        # Run all validations
        self.validate_environment()
        self.validate_jwt_configuration()
        self.validate_oauth_configuration()
        self.validate_cors_configuration()
        self.validate_security_configuration()
        self.validate_database_configuration()
        self.validate_rate_limiting()
        self.validate_external_services()

        # Print summary
        success = self.print_summary()

        return 0 if success else 1


def main():
    """Main entry point."""
    validator = ProductionValidator()
    return validator.run()


if __name__ == "__main__":
    sys.exit(main())
