#!/usr/bin/env python3
"""
Validate extracted code examples in ROS 2 environment.
Runs Python examples and attempts to compile C++ examples.

Usage:
    python validate-code-examples.py [--verbose] [--timeout SECONDS]
"""

import argparse
import json
import logging
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Any, List

import yaml

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)
logger = logging.getLogger(__name__)


def load_config() -> dict:
    """Load configuration from config.yaml."""
    config_path = Path(__file__).parent / "config.yaml"
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        logger.warning(f"Config file not found at {config_path}, using defaults")
        return {
            "validation": {
                "python_timeout": 10,
                "cpp_timeout": 5,
                "output_truncation": 500
            },
            "paths": {
                "code_examples_dir": "tests/code-examples",
                "code_validation_results": "tests/code-validation-results.json"
            }
        }
    except yaml.YAMLError as e:
        logger.error(f"Failed to parse config.yaml: {e}")
        sys.exit(2)


def validate_python_example(
    file_path: Path,
    timeout: int = 10,
    max_output: int = 500
) -> Dict[str, Any]:
    """
    Run a Python code example and check for errors.

    Args:
        file_path: Path to the Python file
        timeout: Maximum execution time in seconds
        max_output: Maximum characters to capture from output

    Returns:
        Dictionary with validation results
    """
    result = {
        'file': str(file_path.name),
        'full_path': str(file_path),
        'language': 'python',
        'success': False,
        'error': None,
        'output': None,
        'execution_time': 0.0
    }

    start_time = time.time()

    try:
        # Determine Python command (python3 on Unix, python on Windows)
        python_cmd = 'python3' if sys.platform != 'win32' else 'python'

        process = subprocess.run(
            [python_cmd, str(file_path)],
            capture_output=True,
            text=True,
            timeout=timeout,
            env={**subprocess.os.environ, 'PYTHONDONTWRITEBYTECODE': '1'}
        )

        result['execution_time'] = round(time.time() - start_time, 3)

        if process.returncode == 0:
            result['success'] = True
            result['output'] = process.stdout[:max_output] if process.stdout else None
        else:
            error_msg = process.stderr[:max_output] if process.stderr else f"Exit code: {process.returncode}"
            result['error'] = error_msg

    except subprocess.TimeoutExpired:
        result['error'] = f'Execution timeout ({timeout}s exceeded)'
        result['execution_time'] = timeout
    except FileNotFoundError as e:
        result['error'] = f'Python interpreter not found: {e}'
    except PermissionError:
        result['error'] = 'Permission denied: Cannot execute file'
    except OSError as e:
        result['error'] = f'OS error: {e}'
    except Exception as e:
        result['error'] = f'Unexpected error: {type(e).__name__}: {e}'

    return result


def validate_cpp_example(
    file_path: Path,
    timeout: int = 5,
    max_output: int = 500
) -> Dict[str, Any]:
    """
    Check if C++ code compiles successfully.

    Args:
        file_path: Path to the C++ file
        timeout: Maximum compilation time in seconds
        max_output: Maximum characters to capture from output

    Returns:
        Dictionary with validation results
    """
    result = {
        'file': str(file_path.name),
        'full_path': str(file_path),
        'language': 'cpp',
        'success': False,
        'error': None,
        'warning': None
    }

    try:
        # Basic syntax check using g++
        process = subprocess.run(
            ['g++', '-std=c++17', '-fsyntax-only', '-Wall', str(file_path)],
            capture_output=True,
            text=True,
            timeout=timeout
        )

        if process.returncode == 0:
            result['success'] = True
            # Capture any warnings
            if process.stderr:
                result['warning'] = process.stderr[:max_output]
        else:
            result['error'] = process.stderr[:max_output] if process.stderr else f"Exit code: {process.returncode}"

    except subprocess.TimeoutExpired:
        result['error'] = f'Compilation timeout ({timeout}s exceeded)'
    except FileNotFoundError:
        result['error'] = 'g++ compiler not found. Install with: apt-get install g++ (Linux) or use Visual Studio (Windows)'
    except PermissionError:
        result['error'] = 'Permission denied: Cannot execute compiler'
    except OSError as e:
        result['error'] = f'OS error: {e}'
    except Exception as e:
        result['error'] = f'Unexpected error: {type(e).__name__}: {e}'

    return result


def validate_files(
    code_dir: Path,
    config: dict,
    verbose: bool = False
) -> List[Dict[str, Any]]:
    """
    Validate all code files in the directory.

    Args:
        code_dir: Directory containing code examples
        config: Configuration dictionary
        verbose: Enable verbose output

    Returns:
        List of validation results
    """
    validation_config = config.get("validation", {})
    python_timeout = validation_config.get("python_timeout", 10)
    cpp_timeout = validation_config.get("cpp_timeout", 5)
    max_output = validation_config.get("output_truncation", 500)

    results = []
    supported_extensions = {
        '.py': ('python', validate_python_example, python_timeout),
        '.cpp': ('cpp', validate_cpp_example, cpp_timeout),
        '.cc': ('cpp', validate_cpp_example, cpp_timeout),
        '.cxx': ('cpp', validate_cpp_example, cpp_timeout),
    }

    files = sorted(code_dir.iterdir())

    for file_path in files:
        if not file_path.is_file():
            continue

        ext = file_path.suffix.lower()
        if ext not in supported_extensions:
            if verbose:
                logger.debug(f"Skipping unsupported file type: {file_path.name}")
            continue

        lang_name, validator_func, timeout = supported_extensions[ext]

        logger.info(f"Validating [{lang_name}]: {file_path.name}")

        result = validator_func(file_path, timeout=timeout, max_output=max_output)
        results.append(result)

        if result['success']:
            status = "PASS"
            if result.get('warning'):
                status += " (with warnings)"
            logger.info(f"  Result: {status}")
        else:
            logger.error(f"  Result: FAIL - {result['error']}")

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Validate extracted code examples for syntax and execution."
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--input", "-i",
        type=Path,
        help="Input directory containing code examples (default: from config)"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        help="Output JSON file for results (default: from config)"
    )
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Load configuration
    config = load_config()
    paths_config = config.get("paths", {})

    # Set up directories
    code_dir = args.input or Path(paths_config.get("code_examples_dir", "tests/code-examples"))
    results_file = args.output or Path(paths_config.get("code_validation_results", "tests/code-validation-results.json"))

    # Validate input directory exists
    if not code_dir.exists():
        logger.error(f"Code examples directory does not exist: {code_dir}")
        logger.info("Run extract-code-examples.py first to extract testable code")
        sys.exit(1)

    if not code_dir.is_dir():
        logger.error(f"Not a directory: {code_dir}")
        sys.exit(1)

    # Run validation
    print(f"\n{'='*60}")
    print("Code Example Validation")
    print(f"{'='*60}")
    print(f"Source: {code_dir}")
    print(f"{'='*60}\n")

    results = validate_files(code_dir, config, verbose=args.verbose)

    if not results:
        logger.warning("No code files found to validate")
        sys.exit(0)

    # Calculate summary statistics
    total = len(results)
    successful = sum(1 for r in results if r['success'])
    failed = total - successful
    warnings = sum(1 for r in results if r.get('warning'))

    # Prepare output data
    output_data = {
        'summary': {
            'total': total,
            'successful': successful,
            'failed': failed,
            'warnings': warnings,
            'success_rate': round((successful / total * 100), 1) if total > 0 else 0,
            'source_directory': str(code_dir),
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        },
        'results': results
    }

    # Save results
    try:
        results_file.parent.mkdir(parents=True, exist_ok=True)
        with open(results_file, 'w', encoding='utf-8') as f:
            json.dump(output_data, f, indent=2)
        logger.info(f"Results saved to: {results_file}")
    except IOError as e:
        logger.error(f"Failed to save results: {e}")

    # Print summary
    print(f"\n{'='*60}")
    print("Validation Summary")
    print(f"{'='*60}")
    print(f"  Total files:    {total}")
    print(f"  Successful:     {successful}")
    print(f"  Failed:         {failed}")
    print(f"  With warnings:  {warnings}")
    print(f"  Success rate:   {output_data['summary']['success_rate']}%")
    print(f"{'='*60}")

    # Exit with appropriate code
    if failed > 0:
        logger.error(f"{failed} code example(s) failed validation")
        sys.exit(1)
    else:
        logger.info("All code examples passed validation")
        sys.exit(0)


if __name__ == "__main__":
    main()