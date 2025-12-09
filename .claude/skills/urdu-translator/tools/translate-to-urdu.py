#!/usr/bin/env python3
"""
Translate MDX content to Urdu while preserving technical terms and code blocks.
"""

import os
import re
import json
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import openai
from openai import OpenAI


@dataclass
class TranslationResult:
    """Result of translation operation."""
    translated_content: str
    original_content: str
    preserved_terms: List[str]
    code_blocks_preserved: int


class MDXPreserver:
    """Handles preservation of MDX elements during translation."""

    def __init__(self, preserve_terms: Optional[List[str]] = None):
        self.preserve_terms = preserve_terms or [
            "ROS", "NVIDIA", "Isaac", "SLAM", "LiDAR", "IMU", "URDF",
            "API", "JSON", "XML", "HTML", "CSS", "JavaScript", "Python",
            "C++", "CUDA", "TensorRT", "Docker", "Kubernetes", "Linux",
            "Ubuntu", "Git", "GitHub", "CLI", "SDK", "IDE", "CPU", "GPU"
        ]
        self.code_blocks = []
        self.inline_codes = []
        self.preserved_placeholders = {}

    def extract_and_preserve(self, content: str) -> Tuple[str, Dict]:
        """Extract elements that should be preserved and replace with placeholders."""
        modified_content = content
        placeholder_map = {}

        # Extract code blocks (```lang...```)
        code_block_pattern = r'```(\w+)?\n(.*?)\n```'
        for match in re.finditer(code_block_pattern, modified_content, re.DOTALL):
            lang = match.group(1) or ""
            code = match.group(2)
            placeholder = f"__CODE_BLOCK_{len(self.code_blocks)}__"
            placeholder_map[placeholder] = {
                "type": "code_block",
                "lang": lang,
                "content": code
            }
            self.code_blocks.append(code)
            modified_content = modified_content.replace(match.group(0), placeholder, 1)

        # Extract inline code (`...`)
        inline_code_pattern = r'`([^`]+)`'
        for match in re.finditer(inline_code_pattern, modified_content):
            code = match.group(1)
            placeholder = f"__INLINE_CODE_{len(self.inline_codes)}__"
            placeholder_map[placeholder] = {
                "type": "inline_code",
                "content": code
            }
            self.inline_codes.append(code)
            modified_content = modified_content.replace(match.group(0), placeholder, 1)

        # Extract URLs and file paths
        url_pattern = r'https?://[^\s<>"{}|\\^`\[\]]+'
        for match in re.finditer(url_pattern, modified_content):
            placeholder = f"__URL_{len(placeholder_map)}__"
            placeholder_map[placeholder] = {
                "type": "url",
                "content": match.group(0)
            }
            modified_content = modified_content.replace(match.group(0), placeholder, 1)

        # Extract file paths
        file_path_pattern = r'(?<!\w)[\w\-./\\]+\.(py|cpp|js|jsx|ts|tsx|yaml|yml|json|xml|html|css|md|txt|urdf|xacro)(?!\w)'
        for match in re.finditer(file_path_pattern, modified_content):
            placeholder = f"__FILEPATH_{len(placeholder_map)}__"
            placeholder_map[placeholder] = {
                "type": "filepath",
                "content": match.group(0)
            }
            modified_content = modified_content.replace(match.group(0), placeholder, 1)

        self.preserved_placeholders = placeholder_map
        return modified_content, placeholder_map

    def restore_preserved(self, translated_content: str, placeholder_map: Dict) -> str:
        """Restore preserved elements from placeholders."""
        restored_content = translated_content

        # Sort placeholders by length to avoid partial replacements
        sorted_placeholders = sorted(placeholder_map.keys(), key=len, reverse=True)

        for placeholder in sorted_placeholders:
            if placeholder in placeholder_map:
                item = placeholder_map[placeholder]
                if item["type"] == "code_block":
                    lang = item["lang"]
                    code = item["content"]
                    replacement = f"```{lang}\n{code}\n```"
                elif item["type"] == "inline_code":
                    replacement = f"`{item['content']}`"
                elif item["type"] in ["url", "filepath"]:
                    replacement = item["content"]
                else:
                    replacement = item["content"]

                restored_content = restored_content.replace(placeholder, replacement, 1)

        return restored_content


class UrduTranslator:
    """Main translator class for MDX to Urdu translation."""

    def __init__(self, api_key: Optional[str] = None):
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OpenAI API key is required")
        self.client = OpenAI(api_key=self.api_key)

    def translate_to_urdu(
        self,
        content: str,
        preserve_terms: Optional[List[str]] = None,
        model: str = "gpt-4"
    ) -> TranslationResult:
        """
        Translate MDX content to Urdu while preserving technical elements.

        Args:
            content: MDX content to translate
            preserve_terms: Additional technical terms to preserve in English
            model: OpenAI model to use (default: gpt-4)

        Returns:
            TranslationResult with translated content and metadata
        """
        preserver = MDXPreserver(preserve_terms)

        # Extract and preserve elements
        content_for_translation, placeholder_map = preserver.extract_and_preserve(content)

        # Create system prompt
        system_prompt = self._create_system_prompt(preserver.preserve_terms)

        # Prepare user prompt with content
        user_prompt = f"""Translate the following MDX content to Urdu. The content contains placeholders for code blocks, URLs, and other elements that must remain unchanged:

{content_for_translation}

Requirements:
1. Translate to natural-sounding Urdu
2. Keep all placeholders unchanged (e.g., __CODE_BLOCK_0__, __URL_1__)
3. Use formal Urdu suitable for technical documentation
4. Add appropriate RTL (right-to-left) formatting
5. Translate image alt text but keep technical accuracy
6. Use appropriate Urdu technical terminology where it exists
"""

        try:
            # Make API call
            response = self.client.chat.completions.create(
                model=model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.3,
                max_tokens=4000
            )

            translated_with_placeholders = response.choices[0].message.content

            # Restore preserved elements
            final_translation = preserver.restore_preserved(
                translated_with_placeholders,
                placeholder_map
            )

            # Add RTL direction to the content
            final_translation = self._add_rtl_support(final_translation)

            return TranslationResult(
                translated_content=final_translation,
                original_content=content,
                preserved_terms=preserver.preserve_terms,
                code_blocks_preserved=len(preserver.code_blocks)
            )

        except Exception as e:
            raise RuntimeError(f"Translation failed: {str(e)}")

    def _create_system_prompt(self, preserve_terms: List[str]) -> str:
        """Create system prompt for the translation task."""
        terms_str = ", ".join(preserve_terms)
        return f"""You are a professional translator specializing in technical documentation. Your task is to translate English MDX content to Urdu while maintaining technical accuracy.

Key guidelines:
1. Translate to formal Urdu suitable for technical documentation
2. Keep the following terms in English: {terms_str}
3. Preserve all placeholder elements (they start and end with __)
4. Use appropriate Urdu terminology for technical concepts
5. Maintain the original formatting, including markdown syntax
6. Translate image alt text accurately
7. Add RTL (right-to-left) direction indicators where appropriate

The output must be valid MDX that can be rendered directly."""

    def _add_rtl_support(self, content: str) -> str:
        """Add RTL support to the translated content."""
        # Check if content already has RTL markers
        if 'dir="rtl"' in content:
            return content

        # Add RTL direction to the main content
        # We wrap the content in a div with RTL direction
        lines = content.split('\n')
        result = []
        frontmatter_end = -1

        # Find end of frontmatter
        for i, line in enumerate(lines):
            if line.strip() == '---' and i > 0:
                frontmatter_end = i
                break

        # Insert RTL directive after frontmatter
        if frontmatter_end > 0:
            result.extend(lines[:frontmatter_end + 1])
            result.append('')
            result.append('<div dir="rtl">')
            result.extend(lines[frontmatter_end + 1:])
            result.append('</div>')
        else:
            result.append('<div dir="rtl">')
            result.extend(lines)
            result.append('</div>')

        return '\n'.join(result)


def translate_to_urdu(
    content: str,
    preserve_terms: Optional[List[str]] = None,
    api_key: Optional[str] = None,
    model: str = "gpt-4"
) -> Dict:
    """
    Main function to translate MDX content to Urdu.

    Args:
        content: MDX content to translate
        preserve_terms: List of technical terms to preserve
        api_key: OpenAI API key
        model: OpenAI model to use

    Returns:
        Dictionary with translation result
    """
    try:
        translator = UrduTranslator(api_key)
        result = translator.translate_to_urdu(content, preserve_terms, model)

        return {
            "success": True,
            "translated_content": result.translated_content,
            "original_content": result.original_content,
            "metadata": {
                "preserved_terms": result.preserved_terms,
                "code_blocks_preserved": result.code_blocks_preserved
            }
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "translated_content": None,
            "original_content": content
        }


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("Usage: python translate-to-urdu.py <input_file> [output_file]")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    try:
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
            print(f"Translation failed: {result['error']}")
            sys.exit(1)

    except Exception as e:
        print(f"Error: {str(e)}")
        sys.exit(1)