"""
Urdu translation tool for MDX content.
Preserves code blocks, technical terms, and MDX syntax while translating prose to Urdu.
"""

import os
import re
import hashlib
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field

from openai import OpenAI


@dataclass
class TranslationResult:
    """Result of a translation operation."""
    translated_content: str
    original_content: str
    preserved_terms: List[str]
    code_blocks_preserved: int
    word_count: int = 0
    cache_hit: bool = False


@dataclass
class PlaceholderInfo:
    """Information about a preserved placeholder."""
    content: str
    type: str  # 'code_block', 'inline_code', 'url', 'filepath', 'term'
    lang: Optional[str] = None


class MDXPreserver:
    """
    Preserves MDX elements that should not be translated.
    Extracts code blocks, inline code, URLs, file paths, and technical terms.
    """

    # Default technical terms to preserve (robotics/ROS specific)
    DEFAULT_PRESERVE_TERMS = [
        "ROS", "ROS 2", "Python", "C++", "API", "SDK", "Gazebo",
        "URDF", "Isaac Sim", "NVIDIA", "OpenCV", "TensorFlow",
        "PyTorch", "NumPy", "SciPy", "Matplotlib", "Docker",
        "Git", "GitHub", "Linux", "Ubuntu", "Raspberry Pi"
    ]

    def __init__(self, preserve_terms: Optional[List[str]] = None):
        """
        Initialize the preserver.

        Args:
            preserve_terms: List of technical terms to preserve in English
        """
        self.preserve_terms = preserve_terms or self.DEFAULT_PRESERVE_TERMS

        # Regex patterns
        self.code_block_pattern = re.compile(
            r'```(\w*)\n([\s\S]*?)```',
            re.MULTILINE
        )
        self.inline_code_pattern = re.compile(r'`([^`]+)`')
        self.url_pattern = re.compile(
            r'https?://[^\s<>"{}|\\^`\[\]]+',
            re.IGNORECASE
        )
        self.filepath_pattern = re.compile(
            r'(?:^|\s)([./]?(?:[\w-]+/)+[\w.-]+\.\w+)',
            re.MULTILINE
        )

    def extract_and_preserve(
        self,
        content: str
    ) -> Tuple[str, Dict[str, PlaceholderInfo]]:
        """
        Extract and replace elements that should be preserved.

        Args:
            content: Original MDX content

        Returns:
            Tuple of (content with placeholders, placeholder map)
        """
        placeholder_map = {}
        modified = content

        # Extract code blocks
        code_block_idx = 0
        for match in self.code_block_pattern.finditer(content):
            placeholder = f"__CODE_BLOCK_{code_block_idx}__"
            placeholder_map[placeholder] = PlaceholderInfo(
                content=match.group(2),
                type='code_block',
                lang=match.group(1) or 'text'
            )
            modified = modified.replace(match.group(0), placeholder, 1)
            code_block_idx += 1

        # Extract inline code
        inline_idx = 0
        for match in self.inline_code_pattern.finditer(modified):
            placeholder = f"__INLINE_CODE_{inline_idx}__"
            placeholder_map[placeholder] = PlaceholderInfo(
                content=match.group(1),
                type='inline_code'
            )
            modified = modified.replace(match.group(0), placeholder, 1)
            inline_idx += 1

        # Extract URLs
        url_idx = 0
        for match in self.url_pattern.finditer(modified):
            placeholder = f"__URL_{url_idx}__"
            placeholder_map[placeholder] = PlaceholderInfo(
                content=match.group(0),
                type='url'
            )
            modified = modified.replace(match.group(0), placeholder, 1)
            url_idx += 1

        # Extract file paths
        filepath_idx = 0
        for match in self.filepath_pattern.finditer(modified):
            placeholder = f"__FILEPATH_{filepath_idx}__"
            placeholder_map[placeholder] = PlaceholderInfo(
                content=match.group(1),
                type='filepath'
            )
            modified = modified.replace(match.group(1), placeholder, 1)
            filepath_idx += 1

        # Mark technical terms (case-insensitive but preserve original case)
        term_idx = 0
        for term in self.preserve_terms:
            pattern = re.compile(rf'\b{re.escape(term)}\b', re.IGNORECASE)
            for match in pattern.finditer(modified):
                placeholder = f"__TERM_{term_idx}__"
                placeholder_map[placeholder] = PlaceholderInfo(
                    content=match.group(0),
                    type='term'
                )
                modified = modified.replace(match.group(0), placeholder, 1)
                term_idx += 1

        return modified, placeholder_map

    def restore_preserved(
        self,
        content: str,
        placeholder_map: Dict[str, PlaceholderInfo]
    ) -> str:
        """
        Restore preserved elements from placeholders.

        Args:
            content: Content with placeholders
            placeholder_map: Map of placeholders to original content

        Returns:
            Content with restored elements
        """
        restored = content

        for placeholder, info in placeholder_map.items():
            if info.type == 'code_block':
                replacement = f"```{info.lang or ''}\n{info.content}```"
            elif info.type == 'inline_code':
                replacement = f"`{info.content}`"
            else:
                replacement = info.content

            restored = restored.replace(placeholder, replacement)

        return restored


class UrduTranslator:
    """
    Translates technical content to Urdu using OpenAI GPT-4.
    Preserves code, technical terms, and MDX syntax.
    """

    SYSTEM_PROMPT = """You are a professional translator specializing in technical documentation translation from English to Urdu.

Your task is to translate the following text while:
1. Maintaining the original meaning and technical accuracy
2. NOT translating any placeholders (text like __CODE_BLOCK_0__, __INLINE_CODE_1__, etc.)
3. Using formal Urdu suitable for educational content
4. Keeping the translation natural and readable
5. Preserving all markdown formatting (headings, lists, links, etc.)

Important:
- Do NOT translate placeholder tokens (they start and end with double underscores)
- Do NOT translate markdown syntax characters (#, *, -, etc.)
- Do NOT add or remove content
- Maintain paragraph structure
- Use right-to-left appropriate punctuation where needed"""

    def __init__(self, api_key: Optional[str] = None):
        """
        Initialize the translator.

        Args:
            api_key: OpenAI API key (uses OPENAI_API_KEY env var if not provided)
        """
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY")

        if not self.api_key:
            raise ValueError(
                "OpenAI API key is required. Set OPENAI_API_KEY environment "
                "variable or pass api_key parameter."
            )

        self.client = OpenAI(api_key=self.api_key)
        self.preserver = MDXPreserver()

    def translate_to_urdu(
        self,
        content: str,
        preserve_terms: Optional[List[str]] = None
    ) -> TranslationResult:
        """
        Translate content to Urdu.

        Args:
            content: Original MDX content
            preserve_terms: Additional terms to preserve

        Returns:
            TranslationResult with translated content and metadata
        """
        # Update preserver with custom terms if provided
        if preserve_terms:
            self.preserver.preserve_terms = (
                self.preserver.DEFAULT_PRESERVE_TERMS + preserve_terms
            )

        # Extract and preserve elements
        preserved_content, placeholder_map = self.preserver.extract_and_preserve(
            content
        )

        # Count code blocks
        code_blocks = sum(
            1 for info in placeholder_map.values()
            if info.type == 'code_block'
        )

        # Get preserved terms
        preserved_terms = [
            info.content for info in placeholder_map.values()
            if info.type == 'term'
        ]

        try:
            # Translate using GPT-4
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": self.SYSTEM_PROMPT},
                    {"role": "user", "content": preserved_content}
                ],
                temperature=0.3,  # Lower temperature for more consistent translations
                max_tokens=4000
            )

            translated = response.choices[0].message.content

            # Restore preserved elements
            restored = self.preserver.restore_preserved(translated, placeholder_map)

            # Add RTL support
            final_content = self._add_rtl_support(restored)

            return TranslationResult(
                translated_content=final_content,
                original_content=content,
                preserved_terms=list(set(preserved_terms)),
                code_blocks_preserved=code_blocks,
                word_count=len(content.split())
            )

        except Exception as e:
            raise RuntimeError(f"Translation failed: {str(e)}")

    def _add_rtl_support(self, content: str) -> str:
        """
        Add RTL wrapper to translated content.

        Args:
            content: Translated content

        Returns:
            Content with RTL wrapper
        """
        # Wrap in RTL div
        return f'<div dir="rtl">{content}</div>'


def translate_to_urdu(
    content: str,
    preserve_terms: Optional[List[str]] = None,
    api_key: Optional[str] = None
) -> Dict:
    """
    Convenience function to translate content to Urdu.

    Args:
        content: Original MDX content
        preserve_terms: Additional terms to preserve in English
        api_key: OpenAI API key

    Returns:
        Dictionary with translation results
    """
    try:
        translator = UrduTranslator(api_key=api_key)
        result = translator.translate_to_urdu(content, preserve_terms)

        return {
            "success": True,
            "translated_content": result.translated_content,
            "original_content": result.original_content,
            "preserved_terms": result.preserved_terms,
            "code_blocks_preserved": result.code_blocks_preserved,
            "word_count": result.word_count,
            "cache_hit": result.cache_hit
        }

    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "original_content": content
        }


def generate_content_hash(content: str) -> str:
    """
    Generate a hash for content caching.

    Args:
        content: Content to hash

    Returns:
        SHA-256 hash of the content
    """
    return hashlib.sha256(content.encode()).hexdigest()


# CLI interface
if __name__ == "__main__":
    import sys
    import json

    if len(sys.argv) < 2:
        print("Usage: python translate_to_urdu.py <input_file> [output_file]")
        print("       python translate_to_urdu.py --text 'text to translate'")
        sys.exit(1)

    if sys.argv[1] == "--text":
        text = sys.argv[2] if len(sys.argv) > 2 else ""
        result = translate_to_urdu(text)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else None

        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        result = translate_to_urdu(content)

        if result["success"]:
            if output_file:
                with open(output_file, 'w', encoding='utf-8') as f:
                    f.write(result["translated_content"])
                print(f"Translation saved to {output_file}")
            else:
                print(result["translated_content"])
        else:
            print(f"Error: {result['error']}", file=sys.stderr)
            sys.exit(1)
