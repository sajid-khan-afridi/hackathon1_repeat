#!/usr/bin/env python3
"""
Convert RSA PEM keys to single-line format for environment variables.
"""

import os

def convert_pem_to_single_line(file_path):
    """Convert a PEM file to single-line format with \\n literals."""
    with open(file_path, 'r') as f:
        content = f.read()

    # Replace actual newlines with literal \n
    single_line = content.replace('\n', '\\n')

    # Remove trailing \\n if present
    if single_line.endswith('\\n'):
        single_line = single_line[:-2]

    return single_line

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    # Convert private key
    private_key_path = os.path.join(base_dir, 'jwt_private_production.pem')
    public_key_path = os.path.join(base_dir, 'jwt_public_production.pem')

    print("=" * 80)
    print("RSA KEY CONVERSION FOR ENVIRONMENT VARIABLES")
    print("=" * 80)
    print()

    if os.path.exists(private_key_path):
        private_single = convert_pem_to_single_line(private_key_path)
        print("JWT_PRIVATE_KEY (copy this entire line):")
        print("-" * 80)
        print(private_single)
        print()
        print("-" * 80)
        print()
    else:
        print(f"ERROR: Private key not found at {private_key_path}")
        print()

    if os.path.exists(public_key_path):
        public_single = convert_pem_to_single_line(public_key_path)
        print("JWT_PUBLIC_KEY (copy this entire line):")
        print("-" * 80)
        print(public_single)
        print()
        print("-" * 80)
        print()
    else:
        print(f"ERROR: Public key not found at {public_key_path}")
        print()

    print("=" * 80)
    print("USAGE IN .ENV FILE:")
    print("=" * 80)
    print('JWT_PRIVATE_KEY="<paste private key here>"')
    print('JWT_PUBLIC_KEY="<paste public key here>"')
    print()
    print("IMPORTANT: Make sure to wrap the keys in double quotes!")
    print("=" * 80)

if __name__ == '__main__':
    main()
