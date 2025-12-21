# RSA Key Conversion Guide for Production Deployment

**Feature:** 001-user-auth
**Purpose:** Convert generated RSA keys to single-line format for environment variables
**Date:** 2025-12-20

---

## Generated Production Keys

The following RSA key pair has been generated for production JWT signing:

- **Private Key:** `jwt_private_production.pem` (2048-bit RSA)
- **Public Key:** `jwt_public_production.pem` (2048-bit RSA)
- **Algorithm:** RS256 (RSA Signature with SHA-256)

**SECURITY WARNING:** These keys are NOT committed to version control and must be stored securely in your deployment platform's secret manager.

---

## Conversion Steps

### Windows (PowerShell)

```powershell
# Navigate to project root
cd "D:\GitHub Connected\hackathon1_repeat"

# Convert private key to single-line format
$private = Get-Content jwt_private_production.pem -Raw
$privateSingleLine = $private -replace "`r`n", "\n" -replace "`n", "\n"
Write-Host "JWT_PRIVATE_KEY (copy this entire line):"
Write-Host $privateSingleLine

# Convert public key to single-line format
$public = Get-Content jwt_public_production.pem -Raw
$publicSingleLine = $public -replace "`r`n", "\n" -replace "`n", "\n"
Write-Host "`nJWT_PUBLIC_KEY (copy this entire line):"
Write-Host $publicSingleLine
```

**Output Format:**
```
JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgk...\n-----END PRIVATE KEY-----"
JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhk...\n-----END PUBLIC KEY-----"
```

### Linux/Mac

```bash
# Navigate to project root
cd ~/hackathon1_repeat

# Convert private key to single-line format
echo "JWT_PRIVATE_KEY (copy this entire line):"
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_private_production.pem

# Convert public key to single-line format
echo -e "\nJWT_PUBLIC_KEY (copy this entire line):"
awk 'NF {sub(/\r/, ""); printf "%s\\n",$0;}' jwt_public_production.pem
```

### Alternative Method (Python)

```python
# create_env_keys.py
import pathlib

def convert_key_to_env_format(pem_file: str) -> str:
    """Convert PEM key to single-line format for environment variables."""
    with open(pem_file, 'r') as f:
        content = f.read()
    # Replace newlines with \n literal
    return content.replace('\n', '\\n').replace('\r', '')

# Convert private key
private_key = convert_key_to_env_format('jwt_private_production.pem')
print(f'JWT_PRIVATE_KEY="{private_key}"')

# Convert public key
public_key = convert_key_to_env_format('jwt_public_production.pem')
print(f'\nJWT_PUBLIC_KEY="{public_key}"')
```

**Run:**
```bash
python create_env_keys.py
```

---

## Storing Keys in Deployment Platforms

### Railway

```bash
# Set private key
railway variables set JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgk...\n-----END PRIVATE KEY-----"

# Set public key
railway variables set JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhk...\n-----END PUBLIC KEY-----"

# Set algorithm
railway variables set JWT_ALGORITHM=RS256

# Verify
railway variables | grep JWT
```

### GitHub Secrets (for GitHub Actions)

1. Go to repository: Settings → Secrets and variables → Actions
2. Click "New repository secret"
3. Add secrets:
   - Name: `JWT_PRIVATE_KEY`
   - Value: `-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgk...\n-----END PRIVATE KEY-----`
4. Repeat for `JWT_PUBLIC_KEY`

### Vercel

```bash
# Install Vercel CLI if not already installed
npm i -g vercel

# Set environment variables
vercel env add JWT_PRIVATE_KEY production
# Paste the single-line key when prompted

vercel env add JWT_PUBLIC_KEY production
# Paste the single-line key when prompted
```

### Heroku

```bash
# Set environment variables
heroku config:set JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgk...\n-----END PRIVATE KEY-----"

heroku config:set JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhk...\n-----END PUBLIC KEY-----"

# Verify
heroku config | grep JWT
```

### Docker Secrets

```bash
# Create secret files
echo "-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgk...\n-----END PRIVATE KEY-----" | docker secret create jwt_private_key -

echo "-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhk...\n-----END PUBLIC KEY-----" | docker secret create jwt_public_key -

# Use in docker-compose.yml
# secrets:
#   - jwt_private_key
#   - jwt_public_key
```

---

## Validation

After setting the environment variables, validate they're correctly formatted:

```python
# test_jwt_keys.py
import os
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.backends import default_backend

def validate_jwt_keys():
    """Validate JWT keys are correctly formatted."""

    # Get keys from environment
    private_key_str = os.getenv("JWT_PRIVATE_KEY")
    public_key_str = os.getenv("JWT_PUBLIC_KEY")

    if not private_key_str or not public_key_str:
        print("❌ JWT keys not set in environment")
        return False

    # Convert \n to actual newlines
    private_key_pem = private_key_str.replace("\\n", "\n").encode()
    public_key_pem = public_key_str.replace("\\n", "\n").encode()

    try:
        # Try to load private key
        from cryptography.hazmat.primitives.serialization import load_pem_private_key
        private_key = load_pem_private_key(private_key_pem, password=None, backend=default_backend())
        print("✅ Private key is valid")

        # Try to load public key
        from cryptography.hazmat.primitives.serialization import load_pem_public_key
        public_key = load_pem_public_key(public_key_pem, backend=default_backend())
        print("✅ Public key is valid")

        # Check key size
        key_size = private_key.key_size
        print(f"✅ Key size: {key_size} bits")

        if key_size < 2048:
            print("⚠️  Warning: Key size is less than 2048 bits (not recommended for production)")

        return True

    except Exception as e:
        print(f"❌ Failed to validate keys: {e}")
        return False

if __name__ == "__main__":
    validate_jwt_keys()
```

**Run validation:**
```bash
# Set environment variables first
export JWT_PRIVATE_KEY="-----BEGIN PRIVATE KEY-----\nMIIEvAIBADANBgk...\n-----END PRIVATE KEY-----"
export JWT_PUBLIC_KEY="-----BEGIN PUBLIC KEY-----\nMIIBIjANBgkqhk...\n-----END PUBLIC KEY-----"

# Validate
python test_jwt_keys.py
```

Expected output:
```
✅ Private key is valid
✅ Public key is valid
✅ Key size: 2048 bits
```

---

## Testing JWT Token Generation

Once keys are set in your environment, test JWT token generation:

```python
# test_jwt_generation.py
import os
import jwt
from datetime import datetime, timedelta

def test_jwt_generation():
    """Test JWT token generation with production keys."""

    # Get keys from environment
    private_key_str = os.getenv("JWT_PRIVATE_KEY")
    public_key_str = os.getenv("JWT_PUBLIC_KEY")

    # Convert to actual newlines
    private_key = private_key_str.replace("\\n", "\n")
    public_key = public_key_str.replace("\\n", "\n")

    # Create test payload
    payload = {
        "sub": "test-user-id",
        "email": "test@example.com",
        "iat": datetime.utcnow(),
        "exp": datetime.utcnow() + timedelta(hours=24)
    }

    try:
        # Generate token
        token = jwt.encode(payload, private_key, algorithm="RS256")
        print("✅ Token generated successfully")
        print(f"Token (first 50 chars): {token[:50]}...")

        # Verify token
        decoded = jwt.decode(token, public_key, algorithms=["RS256"])
        print("✅ Token verified successfully")
        print(f"Decoded payload: {decoded}")

        return True

    except Exception as e:
        print(f"❌ JWT generation/verification failed: {e}")
        return False

if __name__ == "__main__":
    test_jwt_generation()
```

**Run test:**
```bash
python test_jwt_generation.py
```

Expected output:
```
✅ Token generated successfully
Token (first 50 chars): eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ...
✅ Token verified successfully
Decoded payload: {'sub': 'test-user-id', 'email': 'test@example.com', ...}
```

---

## Troubleshooting

### Issue: "Invalid key format"

**Cause:** Key not properly converted to single-line format

**Solution:**
```bash
# Ensure \n is used as literal string, not actual newlines
# Correct:   "-----BEGIN PRIVATE KEY-----\nMIIE...\n-----END PRIVATE KEY-----"
# Incorrect: "-----BEGIN PRIVATE KEY-----
#             MIIE...
#             -----END PRIVATE KEY-----"
```

### Issue: "Key doesn't match"

**Cause:** Private and public keys are from different key pairs

**Solution:**
```bash
# Regenerate keys together
openssl genrsa -out jwt_private_production.pem 2048
openssl rsa -in jwt_private_production.pem -pubout -out jwt_public_production.pem

# Ensure you use both keys from the same generation
```

### Issue: "Token verification fails"

**Cause:** Environment variable has extra spaces or quotes

**Solution:**
```bash
# Check for extra quotes or spaces
echo $JWT_PRIVATE_KEY | head -c 50

# Should start with: -----BEGIN PRIVATE KEY-----\n
# NOT with: "-----BEGIN PRIVATE KEY-----\n" (extra quotes)
```

### Issue: "Railway/Vercel not accepting multi-line value"

**Cause:** Platform expecting single-line value

**Solution:**
```bash
# Use the converted single-line format with \n as literal string
# The backslash-n (\n) should be visible as two characters, not a newline
```

---

## Security Best Practices

1. **Never commit keys to version control**
   - Keys are in `.gitignore` as `*.pem`
   - Always verify with `git status` before committing

2. **Use different keys for each environment**
   - Development: One key pair
   - Staging: Different key pair
   - Production: Different key pair (this one)

3. **Rotate keys regularly**
   - Recommended: Every 6 months
   - After rotation: Revoke all existing tokens

4. **Store keys in encrypted secret managers**
   - Railway Secrets
   - GitHub Secrets
   - AWS Secrets Manager
   - HashiCorp Vault

5. **Backup keys securely**
   - Store backup in password manager (1Password, LastPass)
   - Never store in plain text files
   - Never share via email or chat

6. **Monitor key usage**
   - Log JWT token generation and validation
   - Alert on unusual patterns
   - Track token expiration and refresh rates

---

## Appendix: Current Production Keys

**IMPORTANT:** The actual key values are in the `.pem` files in the project root.

**Files:**
- `jwt_private_production.pem` - Private key (2048-bit RSA)
- `jwt_public_production.pem` - Public key (2048-bit RSA)

**Key Information:**
- Algorithm: RS256
- Key Size: 2048 bits
- Format: PKCS#8 (for private), SPKI (for public)
- Generated: 2025-12-20

**Next Steps:**
1. Convert keys using instructions above
2. Store in Railway/deployment platform secret manager
3. Delete local `.pem` files after deployment (optional but recommended)
4. Test JWT generation in production environment
5. Schedule key rotation in 6 months (June 2026)

---

**Document Version:** 1.0
**Last Updated:** 2025-12-20
**Author:** Infrastructure & DevOps Team
