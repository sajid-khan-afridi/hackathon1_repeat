"""
Utility functions for MDX processing and preservation.
"""

import re
from typing import Dict, List, Tuple, Optional


class MDXProcessor:
    """Handles MDX-specific elements during translation."""

    def __init__(self):
        self.jsx_components = []
        self.imports = []
        self.exports = []
        self.frontmatter = {}

    def extract_frontmatter(self, content: str) -> Tuple[str, Dict, str]:
        """
        Extract and preserve frontmatter from MDX content.

        Returns:
            Tuple of (content_without_frontmatter, frontmatter_dict, raw_frontmatter)
        """
        frontmatter_pattern = r'^---\n(.*?)\n---'
        match = re.match(frontmatter_pattern, content, re.DOTALL)

        if match:
            raw_frontmatter = match.group(0)
            frontmatter_content = match.group(1)

            # Parse YAML frontmatter
            frontmatter_dict = self._parse_yaml(frontmatter_content)

            # Remove frontmatter from content
            content_without_frontmatter = content[match.end():].lstrip('\n')

            return content_without_frontmatter, frontmatter_dict, raw_frontmatter

        return content, {}, ""

    def _parse_yaml(self, yaml_content: str) -> Dict:
        """Simple YAML parser for frontmatter."""
        result = {}
        lines = yaml_content.split('\n')

        for line in lines:
            line = line.strip()
            if line and ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()

                # Remove quotes if present
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                elif value.startswith("'") and value.endswith("'"):
                    value = value[1:-1]

                result[key] = value

        return result

    def extract_jsx_components(self, content: str) -> Tuple[str, List[Dict]]:
        """
        Extract JSX components and replace with placeholders.

        Returns:
            Tuple of (content_with_placeholders, list_of_components)
        """
        # Pattern for JSX components: <ComponentName ...props> ...children </ComponentName>
        jsx_pattern = r'<([A-Z][a-zA-Z0-9]*)(.*?)>(.*?)</\1>'

        components = []
        content_with_placeholders = content

        for match in re.finditer(jsx_pattern, content, re.DOTALL):
            component_name = match.group(1)
            props = match.group(2)
            children = match.group(3)

            placeholder = f"__JSX_COMPONENT_{len(components)}__"
            components.append({
                "name": component_name,
                "props": props,
                "children": children,
                "full_match": match.group(0)
            })

            content_with_placeholders = content_with_placeholders.replace(
                match.group(0),
                placeholder,
                1
            )

        # Also handle self-closing components
        self_closing_pattern = r'<([A-Z][a-zA-Z0-9]*)(.*?)\s*/>'

        for match in re.finditer(self_closing_pattern, content_with_placeholders):
            component_name = match.group(1)
            props = match.group(2)

            placeholder = f"__JSX_SELF_CLOSING_{len(components)}__"
            components.append({
                "name": component_name,
                "props": props,
                "children": None,
                "full_match": match.group(0),
                "self_closing": True
            })

            content_with_placeholders = content_with_placeholders.replace(
                match.group(0),
                placeholder,
                1
            )

        return content_with_placeholders, components

    def restore_jsx_components(self, content: str, components: List[Dict]) -> str:
        """Restore JSX components from placeholders."""
        restored_content = content

        for i, component in enumerate(components):
            if component.get("self_closing"):
                placeholder = f"__JSX_SELF_CLOSING_{i}__"
                replacement = f"<{component['name']}{component['props']}/>"
            else:
                placeholder = f"__JSX_COMPONENT_{i}__"
                replacement = f"<{component['name']}{component['props']}>{component['children']}</{component['name']}>"

            restored_content = restored_content.replace(placeholder, replacement, 1)

        return restored_content

    def extract_imports_exports(self, content: str) -> Tuple[str, List[str], List[str]]:
        """
        Extract import and export statements.

        Returns:
            Tuple of (content_without_imports_exports, imports, exports)
        """
        # Extract imports
        import_pattern = r'^import\s+.*?;?\s*$'
        imports = []

        # Extract exports
        export_pattern = r'^export\s+.*?;?\s*$'
        exports = []

        lines = content.split('\n')
        filtered_lines = []

        for line in lines:
            if re.match(import_pattern, line.strip()):
                imports.append(line.strip())
            elif re.match(export_pattern, line.strip()):
                exports.append(line.strip())
            else:
                filtered_lines.append(line)

        content_without_imports_exports = '\n'.join(filtered_lines)

        return content_without_imports_exports, imports, exports

    def restore_imports_exports(
        self,
        content: str,
        imports: List[str],
        exports: List[str],
        frontmatter: str
    ) -> str:
        """Restore imports and exports to content."""
        lines = content.split('\n')

        # Add imports at the beginning (after frontmatter if present)
        if imports:
            lines.insert(0, '')
            for imp in reversed(imports):
                lines.insert(0, imp)

        # Add exports at the end
        if exports:
            lines.append('')
            lines.extend(exports)

        # Add frontmatter at the very beginning
        if frontmatter:
            lines.insert(0, frontmatter)

        return '\n'.join(lines)

    def add_rtl_to_components(self, content: str) -> str:
        """Add RTL support to common MDX components."""
        # Common components that need RTL support
        rtl_components = ['p', 'div', 'span', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6']

        for component in rtl_components:
            # Add dir="rtl" to components that don't have it
            pattern = rf'<{component}([^>]*)>'
            replacement = f'<{component}\\1 dir="rtl">'

            # Make sure we don't add dir twice
            content = re.sub(
                pattern,
                lambda m: f'<{component}{m.group(1)} dir="rtl">' if 'dir=' not in m.group(1) else m.group(0),
                content
            )

        return content


class ImageProcessor:
    """Handles image processing for translation."""

    def __init__(self):
        self.images = []

    def extract_images(self, content: str) -> Tuple[str, List[Dict]]:
        """
        Extract images and their alt text.

        Returns:
            Tuple of (content_with_placeholders, list_of_images)
        """
        # Pattern for markdown images: ![alt text](url)
        image_pattern = r'!\[([^\]]*)\]\(([^)]+)\)'

        images = []
        content_with_placeholders = content

        for match in re.finditer(image_pattern, content):
            alt_text = match.group(1)
            url = match.group(2)

            placeholder = f"__IMAGE_{len(images)}__"
            images.append({
                "alt_text": alt_text,
                "url": url,
                "full_match": match.group(0)
            })

            content_with_placeholders = content_with_placeholders.replace(
                match.group(0),
                placeholder,
                1
            )

        # Also handle JSX <img> tags
        jsx_img_pattern = r'<img([^>]*?)\s+alt=["\']([^"\']*)["\']([^>]*?)>'

        for match in re.finditer(jsx_img_pattern, content_with_placeholders):
            before_alt = match.group(1)
            alt_text = match.group(2)
            after_alt = match.group(3)

            placeholder = f"__JSX_IMG_{len(images)}__"
            images.append({
                "alt_text": alt_text,
                "url": None,  # URL might be in src attribute
                "before_alt": before_alt,
                "after_alt": after_alt,
                "full_match": match.group(0),
                "jsx": True
            })

            content_with_placeholders = content_with_placeholders.replace(
                match.group(0),
                placeholder,
                1
            )

        return content_with_placeholders, images

    def restore_images(self, content: str, images: List[Dict]) -> str:
        """Restore images with translated alt text."""
        restored_content = content

        for i, image in enumerate(images):
            if image.get("jsx"):
                placeholder = f"__JSX_IMG_{i}__"
                replacement = f'<img{image["before_alt"]} alt="{image["alt_text"]}"{image["after_alt"]}>'
            else:
                placeholder = f"__IMAGE_{i}__"
                replacement = f'![{image["alt_text"]}]({image["url"]})'

            restored_content = restored_content.replace(placeholder, replacement, 1)

        return restored_content