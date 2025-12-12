#!/usr/bin/env python3
"""
Validate extracted code examples in ROS 2 environment.
Runs Python examples and attempts to compile C++ examples.
"""

import os
import subprocess
import sys
import time
from pathlib import Path
import json

def validate_python_example(file_path: Path) -> dict:
    """Run a Python code example and check for errors."""
    result = {
        'file': str(file_path),
        'language': 'python',
        'success': False,
        'error': None,
        'output': None,
        'execution_time': 0
    }

    try:
        # Record start time
        start_time = time.time()

        # Run Python script with timeout
        process = subprocess.run(
            ['python3', str(file_path)],
            capture_output=True,
            text=True,
            timeout=10  # 10 second timeout
        )

        # Calculate execution time
        result['execution_time'] = time.time() - start_time

        if process.returncode == 0:
            result['success'] = True
            result['output'] = process.stdout[:500]  # Limit output length
        else:
            result['error'] = process.stderr[:500]

    except subprocess.TimeoutExpired:
        result['error'] = 'Execution timeout (10s)'
    except Exception as e:
        result['error'] = str(e)

    return result

def validate_cpp_example(file_path: Path) -> dict:
    """Check if C++ code compiles successfully."""
    result = {
        'file': str(file_path),
        'language': 'cpp',
        'success': False,
        'error': None,
        'warning': None
    }

    try:
        # Basic syntax check using g++
        process = subprocess.run(
            ['g++', '-std=c++17', '-fsyntax-only', str(file_path)],
            capture_output=True,
            text=True,
            timeout=5
        )

        if process.returncode == 0:
            result['success'] = True
        else:
            result['error'] = process.stderr[:500]

    except subprocess.TimeoutExpired:
        result['error'] = 'Compilation timeout'
    except FileNotFoundError:
        result['error'] = 'g++ compiler not found'
    except Exception as e:
        result['error'] = str(e)

    return result

def main():
    code_dir = Path("tests/code-examples")
    if not code_dir.exists():
        print(f"Error: Code examples directory {code_dir} does not exist")
        print("Run extract-code-examples.py first")
        sys.exit(1)

    results = []
    total_files = 0
    successful = 0
    failed = 0

    print("🔍 Validating code examples...\n")

    for file_path in code_dir.iterdir():
        if not file_path.is_file():
            continue

        total_files += 1
        print(f"Validating: {file_path.name}", end=" ... ")

        if file_path.suffix == '.py':
            result = validate_python_example(file_path)
        elif file_path.suffix in ['.cpp', '.cc', '.cxx']:
            result = validate_cpp_example(file_path)
        else:
            print(f"⚠️  Skipped (unknown file type)")
            continue

        results.append(result)

        if result['success']:
            print("✅ PASS")
            successful += 1
        else:
            print(f"❌ FAIL")
            failed += 1
            print(f"   Error: {result['error']}")

    # Save results to JSON
    results_file = Path("tests/code-validation-results.json")
    with open(results_file, 'w', encoding='utf-8') as f:
        json.dump({
            'summary': {
                'total': total_files,
                'successful': successful,
                'failed': failed,
                'success_rate': (successful / total_files * 100) if total_files > 0 else 0
            },
            'results': results
        }, f, indent=2)

    # Print summary
    print(f"\n📊 Validation Summary:")
    print(f"   Total files: {total_files}")
    print(f"   ✅ Successful: {successful}")
    print(f"   ❌ Failed: {failed}")
    print(f"   Success Rate: {(successful/total_files*100):.1f}%" if total_files > 0 else "0%")
    print(f"\n📄 Detailed results saved to: {results_file}")

    # Exit with error code if any failures
    if failed > 0:
        print("\n❌ Some code examples failed validation")
        sys.exit(1)
    else:
        print("\n✅ All code examples passed validation")
        sys.exit(0)

if __name__ == "__main__":
    main()