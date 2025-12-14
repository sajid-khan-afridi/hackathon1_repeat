#!/usr/bin/env python3
"""
Extract code examples from MDX files for validation.
Finds all code blocks marked with # test: true and saves them to individual files.

Usage:
    python extract-code-examples.py <docs_directory> [--verbose]
"""

import argparse
import logging
import re
import sys
from pathlib import Path
from typing import Tuple

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
            "extraction": {"test_marker": "# test: true"},
            "paths": {"code_examples_dir": "tests/code-examples"}
        }
    except yaml.YAMLError as e:
        logger.error(f"Failed to parse config.yaml: {e}")
        sys.exit(2)


def validate_output_directory(output_dir: Path) -> None:
    """Ensure output directory exists and is writable."""
    try:
        output_dir.mkdir(parents=True, exist_ok=True)
        # Test write permission by creating and removing a temp file
        test_file = output_dir / ".write_test"
        test_file.touch()
        test_file.unlink()
    except PermissionError:
        logger.error(f"Permission denied: Cannot write to {output_dir}")
        sys.exit(3)
    except OSError as e:
        logger.error(f"Failed to create output directory {output_dir}: {e}")
        sys.exit(3)


def extract_code_from_mdx(
    mdx_file: Path,
    output_dir: Path,
    test_marker: str = "# test: true"
) -> Tuple[int, int]:
    """
    Extract testable code blocks from an MDX file.

    Returns:
        Tuple of (extracted_count, skipped_count)
    """
    try:
        with open(mdx_file, 'r', encoding='utf-8') as f:
            content = f.read()
    except IOError as e:
        logger.error(f"Failed to read {mdx_file}: {e}")
        return 0, 0

    # More robust regex for code blocks (handles optional info string)
    code_block_pattern = r'```(\w+)?(?:\s+[^\n]*)?\n(.*?)```'
    code_blocks = re.findall(code_block_pattern, content, re.DOTALL)

    extracted_count = 0
    skipped_count = 0

    for i, (lang, code) in enumerate(code_blocks):
        if not lang:
            lang = "txt"

        # Normalize language extension
        lang_ext = {
            "python": "py",
            "javascript": "js",
            "typescript": "ts",
            "cpp": "cpp",
            "c++": "cpp",
        }.get(lang.lower(), lang.lower())

        lines = code.strip().split('\n')
        if not lines:
            continue

        # Check if code block is marked for testing
        first_line = lines[0].strip().lower()
        if test_marker.lower() not in first_line:
            skipped_count += 1
            continue

        # Create filename based on chapter and block number
        chapter_name = mdx_file.stem
        filename = f"{chapter_name}_{i+1}.{lang_ext}"
        output_path = output_dir / filename

        # Remove the test marker line from the code
        if test_marker.lower() in first_line:
            code_to_write = '\n'.join(lines[1:])
        else:
            code_to_write = '\n'.join(lines)

        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(code_to_write.strip() + '\n')
            logger.info(f"Extracted: {output_path.name}")
            extracted_count += 1
        except IOError as e:
            logger.error(f"Failed to write {output_path}: {e}")

    return extracted_count, skipped_count


def main():
    parser = argparse.ArgumentParser(
        description="Extract testable code examples from MDX files."
    )
    parser.add_argument(
        "docs_directory",
        type=Path,
        help="Directory containing MDX files"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        help="Output directory for extracted code (default: from config)"
    )
    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Load configuration
    config = load_config()

    # Validate input directory
    docs_dir = args.docs_directory
    if not docs_dir.exists():
        logger.error(f"Directory does not exist: {docs_dir}")
        sys.exit(1)
    if not docs_dir.is_dir():
        logger.error(f"Not a directory: {docs_dir}")
        sys.exit(1)

    # Set up output directory
    output_dir = args.output or Path(config["paths"]["code_examples_dir"])
    validate_output_directory(output_dir)

    test_marker = config.get("extraction", {}).get("test_marker", "# test: true")

    total_extracted = 0
    total_skipped = 0
    files_processed = 0

    logger.info(f"Scanning {docs_dir} for MDX files...")

    # Find all MDX files
    mdx_files = list(docs_dir.rglob("*.mdx"))

    if not mdx_files:
        logger.warning(f"No MDX files found in {docs_dir}")
        sys.exit(0)

    logger.info(f"Found {len(mdx_files)} MDX files")

    for mdx_file in mdx_files:
        logger.debug(f"Processing: {mdx_file}")
        extracted, skipped = extract_code_from_mdx(mdx_file, output_dir, test_marker)
        total_extracted += extracted
        total_skipped += skipped
        if extracted > 0:
            files_processed += 1

    # Summary
    print(f"\n{'='*50}")
    print(f"Extraction Summary")
    print(f"{'='*50}")
    print(f"  MDX files scanned: {len(mdx_files)}")
    print(f"  Files with tests:  {files_processed}")
    print(f"  Code blocks extracted: {total_extracted}")
    print(f"  Code blocks skipped:   {total_skipped}")
    print(f"  Output directory: {output_dir}")
    print(f"{'='*50}")

    if total_extracted == 0:
        logger.warning("No testable code blocks found (marked with '# test: true')")
        sys.exit(0)

    logger.info(f"Successfully extracted {total_extracted} code examples")
    sys.exit(0)


if __name__ == "__main__":
    main()