#!/usr/bin/env python3
"""
Extract code examples from MDX files for validation.
Finds all code blocks marked with # test: true and saves them to individual files.
"""

import os
import re
import sys
from pathlib import Path

def extract_code_from_mdx(mdx_file: Path, output_dir: Path):
    """Extract testable code blocks from an MDX file."""
    with open(mdx_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Find all code blocks with language specified
    code_blocks = re.findall(r'```(\w+)\n(.*?)\n```', content, re.DOTALL)

    extracted_count = 0
    for i, (lang, code) in enumerate(code_blocks):
        # Check if code block is marked for testing
        if '# test: true' in code.split('\n')[0]:
            # Create filename based on chapter and block number
            chapter_name = mdx_file.stem
            filename = f"{chapter_name}_{i+1}.{lang}"
            output_path = output_dir / filename

            # Remove the test marker line from the code
            lines = code.split('\n')
            if lines[0].strip().startswith('# test: true'):
                code = '\n'.join(lines[1:])

            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(code)

            print(f"Extracted: {output_path}")
            extracted_count += 1

    return extracted_count

def main():
    if len(sys.argv) != 2:
        print("Usage: python extract-code-examples.py <docs_directory>")
        sys.exit(1)

    docs_dir = Path(sys.argv[1])
    if not docs_dir.exists():
        print(f"Error: Directory {docs_dir} does not exist")
        sys.exit(1)

    # Create output directory
    output_dir = Path("tests/code-examples")
    output_dir.mkdir(parents=True, exist_ok=True)

    total_extracted = 0

    # Find all MDX files
    for mdx_file in docs_dir.rglob("*.mdx"):
        print(f"\nProcessing: {mdx_file}")
        count = extract_code_from_mdx(mdx_file, output_dir)
        total_extracted += count

    print(f"\n✅ Extracted {total_extracted} testable code examples to {output_dir}")

if __name__ == "__main__":
    main()