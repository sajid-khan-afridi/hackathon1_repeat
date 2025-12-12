#!/usr/bin/env python3
"""
Validate MDX chapter readability using Flesch-Kincaid Grade Level.
Ensures content is written at appropriate college-level (12-14).
"""

import os
import re
import sys
import json
from pathlib import Path
import textstat

def extract_prose_from_mdx(content: str) -> str:
    """Extract readable text from MDX content, removing code blocks and frontmatter."""
    # Remove frontmatter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove code blocks
    content = re.sub(r'```.*?\n.*?\n```', '', content, flags=re.DOTALL)

    # Remove inline code
    content = re.sub(r'`[^`]+`', '', content)

    # Remove MDX components (e.g., <TechnicalTerm>)
    content = re.sub(r'<[^>]+>[^<]*<\/[^>]+>', ' ', content)
    content = re.sub(r'<[^>]+\/?>', ' ', content)

    # Remove markdown syntax
    content = re.sub(r'#+\s', '', content)  # Headers
    content = re.sub(r'\*\*(.*?)\*\*', r'\1', content)  # Bold
    content = re.sub(r'\*(.*?)\*', r'\1', content)  # Italic
    content = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', content)  # Links
    content = re.sub(r'!\[.*?\]\(.*?\)', '', content)  # Images

    # Clean up whitespace
    content = re.sub(r'\s+', ' ', content).strip()

    return content

def validate_readability(file_path: Path) -> dict:
    """Validate readability of a single MDX file."""
    result = {
        'file': str(file_path),
        'readability_score': 0,
        'passes': False,
        'word_count': 0,
        'sentence_count': 0,
        'error': None
    }

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        prose = extract_prose_from_mdx(content)

        if not prose:
            result['error'] = 'No readable content found'
            return result

        # Calculate readability scores
        result['readability_score'] = textstat.flesch_kincaid_grade(prose)
        result['word_count'] = textstat.lexicon_count(prose, removepunct=True)
        result['sentence_count'] = textstat.sentence_count(prose)

        # Check if within target range (12-14)
        result['passes'] = 12.0 <= result['readability_score'] <= 14.0

    except Exception as e:
        result['error'] = str(e)

    return result

def main():
    if len(sys.argv) < 2:
        print("Usage: python validate-readability.py <mdx_file_or_directory>")
        sys.exit(1)

    target = Path(sys.argv[1])
    results = []

    if target.is_file() and target.suffix == '.mdx':
        files = [target]
    elif target.is_dir():
        files = list(target.rglob("*.mdx"))
    else:
        print(f"Error: {target} is not a valid MDX file or directory")
        sys.exit(1)

    total_files = len(files)
    passing_files = 0
    failing_files = 0

    print("📖 Validating readability (Flesch-Kincaid target: 12-14)...\n")

    for file_path in files:
        print(f"Checking: {file_path.relative_to(Path.cwd())}", end=" ... ")

        result = validate_readability(file_path)
        results.append(result)

        if result['error']:
            print(f"⚠️  ERROR - {result['error']}")
        elif result['passes']:
            print(f"✅ PASS ({result['readability_score']:.1f})")
            passing_files += 1
        else:
            print(f"❌ FAIL ({result['readability_score']:.1f})")
            failing_files += 1

    # Calculate averages
    valid_results = [r for r in results if not r['error']]
    if valid_results:
        avg_score = sum(r['readability_score'] for r in valid_results) / len(valid_results)
        avg_words = sum(r['word_count'] for r in valid_results) / len(valid_results)
    else:
        avg_score = 0
        avg_words = 0

    # Save results to JSON
    results_file = Path("tests/readability-results.json")
    with open(results_file, 'w', encoding='utf-8') as f:
        json.dump({
            'summary': {
                'total_files': total_files,
                'passing': passing_files,
                'failing': failing_files,
                'pass_rate': (passing_files / total_files * 100) if total_files > 0 else 0,
                'average_score': avg_score,
                'average_word_count': avg_words,
                'target_range': '12-14'
            },
            'results': results
        }, f, indent=2)

    # Print summary
    print(f"\n📊 Readability Summary:")
    print(f"   Total files: {total_files}")
    print(f"   ✅ Passing: {passing_files}")
    print(f"   ❌ Failing: {failing_files}")
    print(f"   Pass Rate: {(passing_files/total_files*100):.1f}%" if total_files > 0 else "0%")
    print(f"   Average Score: {avg_score:.1f}")
    print(f"   Average Word Count: {avg_words:.0f}")
    print(f"\n📄 Detailed results saved to: {results_file}")

    # Exit with error code if any failures
    if failing_files > 0:
        print("\n❌ Some files failed readability validation")
        sys.exit(1)
    else:
        print("\n✅ All files passed readability validation")
        sys.exit(0)

if __name__ == "__main__":
    # Ensure textstat is installed
    try:
        import textstat
    except ImportError:
        print("Error: textstat library is not installed")
        print("Install with: pip install textstat")
        sys.exit(1)

    main()