#!/usr/bin/env python3
"""
Validate MDX chapter readability using Flesch-Kincaid Grade Level.
Ensures content is written at appropriate college-level (configurable, default 12-14).

Usage:
    python validate-readability.py <mdx_file_or_directory> [--verbose]
"""

import argparse
import json
import logging
import re
import sys
import time
from pathlib import Path
from typing import Dict, Any, List, Tuple

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
                "readability_range": {"min": 12.0, "max": 14.0}
            },
            "paths": {
                "readability_results": "tests/readability-results.json"
            }
        }
    except yaml.YAMLError as e:
        logger.error(f"Failed to parse config.yaml: {e}")
        sys.exit(2)


def check_dependencies() -> None:
    """Check that required dependencies are installed."""
    try:
        import textstat  # noqa: F401
    except ImportError:
        logger.error("Required dependency 'textstat' is not installed")
        logger.info("Install with: pip install textstat")
        sys.exit(1)


def extract_prose_from_mdx(content: str) -> str:
    """
    Extract readable text from MDX content, removing code blocks and frontmatter.

    Uses improved regex patterns to handle edge cases better.
    """
    # Remove YAML frontmatter (handles both --- and ---)
    content = re.sub(r'^---[\s\S]*?---\s*', '', content.strip())

    # Remove code blocks (handles various formats including info strings)
    # This pattern is more robust and handles nested backticks
    content = re.sub(r'```[\w-]*\s*\n[\s\S]*?```', '', content)

    # Remove inline code (handles backticks)
    content = re.sub(r'`[^`\n]+`', '', content)

    # Remove import/export statements (MDX/JSX)
    content = re.sub(r'^(import|export)\s+.*$', '', content, flags=re.MULTILINE)

    # Remove MDX/JSX components (multi-line and self-closing)
    # Handle self-closing tags first
    content = re.sub(r'<\w+[^>]*/>', ' ', content)
    # Handle tags with content (non-greedy)
    content = re.sub(r'<(\w+)[^>]*>[\s\S]*?</\1>', ' ', content)
    # Clean remaining tag artifacts
    content = re.sub(r'<[^>]+>', ' ', content)

    # Remove markdown syntax while preserving text content
    content = re.sub(r'^#+\s*', '', content, flags=re.MULTILINE)  # Headers
    content = re.sub(r'\*\*([^*]+)\*\*', r'\1', content)  # Bold
    content = re.sub(r'\*([^*]+)\*', r'\1', content)  # Italic
    content = re.sub(r'__([^_]+)__', r'\1', content)  # Bold (underscore)
    content = re.sub(r'_([^_]+)_', r'\1', content)  # Italic (underscore)
    content = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', content)  # Links
    content = re.sub(r'!\[([^\]]*)\]\([^)]+\)', '', content)  # Images
    content = re.sub(r'^>\s*', '', content, flags=re.MULTILINE)  # Blockquotes
    content = re.sub(r'^[-*+]\s+', '', content, flags=re.MULTILINE)  # List items
    content = re.sub(r'^\d+\.\s+', '', content, flags=re.MULTILINE)  # Numbered lists

    # Remove horizontal rules
    content = re.sub(r'^[-*_]{3,}\s*$', '', content, flags=re.MULTILINE)

    # Clean up excessive whitespace while preserving paragraph breaks
    content = re.sub(r'\n{3,}', '\n\n', content)
    content = re.sub(r'[ \t]+', ' ', content)
    content = content.strip()

    return content


def validate_readability(
    file_path: Path,
    min_score: float = 12.0,
    max_score: float = 14.0
) -> Dict[str, Any]:
    """
    Validate readability of a single MDX file.

    Args:
        file_path: Path to the MDX file
        min_score: Minimum acceptable Flesch-Kincaid grade level
        max_score: Maximum acceptable Flesch-Kincaid grade level

    Returns:
        Dictionary with validation results
    """
    import textstat

    result = {
        'file': str(file_path.name),
        'full_path': str(file_path),
        'readability_score': 0.0,
        'passes': False,
        'word_count': 0,
        'sentence_count': 0,
        'error': None,
        'target_range': f"{min_score}-{max_score}"
    }

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except IOError as e:
        result['error'] = f'Failed to read file: {e}'
        return result
    except UnicodeDecodeError as e:
        result['error'] = f'Encoding error: {e}'
        return result

    try:
        prose = extract_prose_from_mdx(content)

        if not prose or len(prose.strip()) < 100:
            result['error'] = 'Insufficient readable content (minimum 100 characters)'
            return result

        # Calculate readability scores
        result['readability_score'] = round(textstat.flesch_kincaid_grade(prose), 2)
        result['word_count'] = textstat.lexicon_count(prose, removepunct=True)
        result['sentence_count'] = textstat.sentence_count(prose)

        # Check if within target range
        result['passes'] = min_score <= result['readability_score'] <= max_score

    except Exception as e:
        result['error'] = f'Analysis error: {type(e).__name__}: {e}'

    return result


def validate_files(
    files: List[Path],
    config: dict,
    verbose: bool = False
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """
    Validate multiple files and return results with summary.

    Args:
        files: List of file paths to validate
        config: Configuration dictionary
        verbose: Enable verbose output

    Returns:
        Tuple of (results_list, summary_dict)
    """
    validation_config = config.get("validation", {})
    readability_range = validation_config.get("readability_range", {})
    min_score = readability_range.get("min", 12.0)
    max_score = readability_range.get("max", 14.0)

    results = []
    passing = 0
    failing = 0
    errors = 0

    for file_path in files:
        logger.info(f"Checking: {file_path.name}")

        result = validate_readability(file_path, min_score, max_score)
        results.append(result)

        if result['error']:
            logger.warning(f"  ERROR: {result['error']}")
            errors += 1
        elif result['passes']:
            logger.info(f"  PASS (score: {result['readability_score']:.1f})")
            passing += 1
        else:
            logger.warning(f"  FAIL (score: {result['readability_score']:.1f}, target: {min_score}-{max_score})")
            failing += 1

    # Calculate averages from valid results
    valid_results = [r for r in results if not r['error']]
    if valid_results:
        avg_score = sum(r['readability_score'] for r in valid_results) / len(valid_results)
        avg_words = sum(r['word_count'] for r in valid_results) / len(valid_results)
        avg_sentences = sum(r['sentence_count'] for r in valid_results) / len(valid_results)
    else:
        avg_score = avg_words = avg_sentences = 0

    summary = {
        'total_files': len(files),
        'passing': passing,
        'failing': failing,
        'errors': errors,
        'pass_rate': round((passing / len(files) * 100), 1) if files else 0,
        'average_score': round(avg_score, 2),
        'average_word_count': round(avg_words),
        'average_sentence_count': round(avg_sentences),
        'target_range': f"{min_score}-{max_score}",
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    }

    return results, summary


def main():
    parser = argparse.ArgumentParser(
        description="Validate MDX chapter readability using Flesch-Kincaid Grade Level."
    )
    parser.add_argument(
        "target",
        type=Path,
        help="MDX file or directory to validate"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        help="Output JSON file for results (default: from config)"
    )
    parser.add_argument(
        "--min-score",
        type=float,
        help="Minimum acceptable readability score (overrides config)"
    )
    parser.add_argument(
        "--max-score",
        type=float,
        help="Maximum acceptable readability score (overrides config)"
    )
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Check dependencies first
    check_dependencies()

    # Load configuration
    config = load_config()
    paths_config = config.get("paths", {})

    # Override config with CLI arguments if provided
    if args.min_score is not None:
        config.setdefault("validation", {}).setdefault("readability_range", {})["min"] = args.min_score
    if args.max_score is not None:
        config.setdefault("validation", {}).setdefault("readability_range", {})["max"] = args.max_score

    # Determine files to validate
    target = args.target
    if not target.exists():
        logger.error(f"Path does not exist: {target}")
        sys.exit(1)

    if target.is_file():
        if target.suffix.lower() != '.mdx':
            logger.error(f"Not an MDX file: {target}")
            sys.exit(1)
        files = [target]
    elif target.is_dir():
        files = sorted(target.rglob("*.mdx"))
        if not files:
            logger.warning(f"No MDX files found in: {target}")
            sys.exit(0)
    else:
        logger.error(f"Invalid path: {target}")
        sys.exit(1)

    # Run validation
    readability_range = config.get("validation", {}).get("readability_range", {})
    min_score = readability_range.get("min", 12.0)
    max_score = readability_range.get("max", 14.0)

    print(f"\n{'='*60}")
    print("Readability Validation")
    print(f"{'='*60}")
    print(f"Target range: Flesch-Kincaid {min_score}-{max_score}")
    print(f"Files to check: {len(files)}")
    print(f"{'='*60}\n")

    results, summary = validate_files(files, config, verbose=args.verbose)

    # Save results
    results_file = args.output or Path(paths_config.get("readability_results", "tests/readability-results.json"))
    try:
        results_file.parent.mkdir(parents=True, exist_ok=True)
        with open(results_file, 'w', encoding='utf-8') as f:
            json.dump({'summary': summary, 'results': results}, f, indent=2)
        logger.info(f"Results saved to: {results_file}")
    except IOError as e:
        logger.error(f"Failed to save results: {e}")

    # Print summary
    print(f"\n{'='*60}")
    print("Validation Summary")
    print(f"{'='*60}")
    print(f"  Total files:     {summary['total_files']}")
    print(f"  Passing:         {summary['passing']}")
    print(f"  Failing:         {summary['failing']}")
    print(f"  Errors:          {summary['errors']}")
    print(f"  Pass rate:       {summary['pass_rate']}%")
    print(f"  Average score:   {summary['average_score']}")
    print(f"  Average words:   {summary['average_word_count']}")
    print(f"{'='*60}")

    # Exit with appropriate code
    if summary['failing'] > 0:
        logger.error(f"{summary['failing']} file(s) failed readability validation")
        sys.exit(1)
    else:
        logger.info("All files passed readability validation")
        sys.exit(0)


if __name__ == "__main__":
    main()