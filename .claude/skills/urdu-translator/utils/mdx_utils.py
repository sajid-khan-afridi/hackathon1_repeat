"""
MDX processing utilities for Urdu translation.
Handles frontmatter, JSX components, and image extraction.
"""

import re
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class JSXComponent:
    """Represents an extracted JSX component."""
    name: str
    full_match: str
    start_index: int
    end_index: int
    self_closing: bool = False
    props: Optional[Dict[str, str]] = None


@dataclass
class ImageInfo:
    """Represents an extracted image."""
    alt_text: str
    url: str
    full_match: str
    jsx: bool = False


class MDXProcessor:
    """
    Processes MDX content for translation.
    Handles frontmatter extraction, JSX components, and RTL support.
    """

    def __init__(self):
        # Patterns for different MDX elements
        self.frontmatter_pattern = re.compile(
            r'^---\n([\s\S]*?)\n---\n?',
            re.MULTILINE
        )
        self.jsx_component_pattern = re.compile(
            r'<([A-Z][a-zA-Z]*)\s*([^>]*?)>([\s\S]*?)</\1>',
            re.MULTILINE
        )
        self.jsx_self_closing_pattern = re.compile(
            r'<([A-Z][a-zA-Z]*)\s*([^/>]*?)\s*/>',
            re.MULTILINE
        )

    def extract_frontmatter(self, content: str) -> Tuple[str, Dict[str, str], str]:
        """
        Extract frontmatter from MDX content.

        Args:
            content: Raw MDX content

        Returns:
            Tuple of (content without frontmatter, parsed frontmatter dict, raw frontmatter)
        """
        match = self.frontmatter_pattern.match(content)

        if not match:
            return content, {}, ""

        raw_frontmatter = match.group(1)
        content_without = content[match.end():]

        # Parse frontmatter into dict
        frontmatter_dict = {}
        for line in raw_frontmatter.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")
                frontmatter_dict[key] = value

        return content_without, frontmatter_dict, raw_frontmatter

    def extract_jsx_components(self, content: str) -> Tuple[str, List[JSXComponent]]:
        """
        Extract JSX components from content, replacing with placeholders.

        Args:
            content: MDX content

        Returns:
            Tuple of (content with placeholders, list of components)
        """
        components = []
        modified_content = content
        placeholder_idx = 0

        # First, handle regular JSX components
        for match in self.jsx_component_pattern.finditer(content):
            component = JSXComponent(
                name=match.group(1),
                full_match=match.group(0),
                start_index=match.start(),
                end_index=match.end(),
                self_closing=False,
                props=self._parse_props(match.group(2))
            )
            components.append(component)
            placeholder = f"__JSX_COMPONENT_{placeholder_idx}__"
            modified_content = modified_content.replace(
                match.group(0),
                placeholder,
                1
            )
            placeholder_idx += 1

        # Then handle self-closing components
        for match in self.jsx_self_closing_pattern.finditer(content):
            # Skip if already captured as part of a component
            if any(match.start() >= c.start_index and match.end() <= c.end_index
                   for c in components if not c.self_closing):
                continue

            component = JSXComponent(
                name=match.group(1),
                full_match=match.group(0),
                start_index=match.start(),
                end_index=match.end(),
                self_closing=True,
                props=self._parse_props(match.group(2))
            )
            components.append(component)
            placeholder = f"__JSX_SELF_CLOSING_{placeholder_idx}__"
            modified_content = modified_content.replace(
                match.group(0),
                placeholder,
                1
            )
            placeholder_idx += 1

        return modified_content, components

    def restore_jsx_components(
        self,
        content: str,
        components: List[JSXComponent]
    ) -> str:
        """
        Restore JSX components from placeholders.

        Args:
            content: Content with placeholders
            components: List of original components

        Returns:
            Content with restored JSX components
        """
        modified = content

        for i, component in enumerate(components):
            if component.self_closing:
                placeholder = f"__JSX_SELF_CLOSING_{i}__"
            else:
                placeholder = f"__JSX_COMPONENT_{i}__"

            modified = modified.replace(placeholder, component.full_match)

        return modified

    def add_rtl_to_components(self, content: str) -> str:
        """
        Add RTL (right-to-left) direction attribute to HTML components.

        Args:
            content: HTML/MDX content

        Returns:
            Content with dir="rtl" added to appropriate tags
        """
        # Tags that should get RTL attribute
        rtl_tags = ['p', 'div', 'span', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6',
                    'li', 'td', 'th', 'blockquote']

        modified = content

        for tag in rtl_tags:
            # Pattern to find tags without dir attribute
            pattern = re.compile(f'<{tag}(?![^>]*dir=)([^>]*)>', re.IGNORECASE)
            modified = pattern.sub(f'<{tag} dir="rtl"\\1>', modified)

        return modified

    def _parse_props(self, props_str: str) -> Dict[str, str]:
        """
        Parse JSX props string into dictionary.

        Args:
            props_str: Raw props string like 'type="warning" onClick={handler}'

        Returns:
            Dictionary of prop names to values
        """
        props = {}
        if not props_str:
            return props

        # Match both string and JSX expression props
        pattern = re.compile(r'(\w+)=(?:"([^"]*)"|{([^}]*)})')

        for match in pattern.finditer(props_str):
            key = match.group(1)
            value = match.group(2) or match.group(3)
            props[key] = value

        return props


class ImageProcessor:
    """
    Processes images in MDX content.
    Handles both Markdown and JSX image formats.
    """

    def __init__(self):
        # Markdown image: ![alt](url)
        self.md_image_pattern = re.compile(
            r'!\[([^\]]*)\]\(([^)]+)\)'
        )
        # JSX img tag: <img src="..." alt="..." />
        self.jsx_image_pattern = re.compile(
            r'<img\s+([^>]*?)/>',
            re.IGNORECASE
        )

    def extract_images(self, content: str) -> Tuple[str, List[ImageInfo]]:
        """
        Extract images from content, replacing with placeholders.

        Args:
            content: MDX content

        Returns:
            Tuple of (content with placeholders, list of images)
        """
        images = []
        modified_content = content
        placeholder_idx = 0

        # Extract Markdown images
        for match in self.md_image_pattern.finditer(content):
            image = ImageInfo(
                alt_text=match.group(1),
                url=match.group(2),
                full_match=match.group(0),
                jsx=False
            )
            images.append(image)
            placeholder = f"__IMAGE_{placeholder_idx}__"
            modified_content = modified_content.replace(
                match.group(0),
                placeholder,
                1
            )
            placeholder_idx += 1

        # Extract JSX images
        for match in self.jsx_image_pattern.finditer(content):
            props_str = match.group(1)
            alt_match = re.search(r'alt="([^"]*)"', props_str)
            src_match = re.search(r'src="([^"]*)"', props_str)

            image = ImageInfo(
                alt_text=alt_match.group(1) if alt_match else "",
                url=src_match.group(1) if src_match else "",
                full_match=match.group(0),
                jsx=True
            )
            images.append(image)
            placeholder = f"__JSX_IMG_{placeholder_idx}__"
            modified_content = modified_content.replace(
                match.group(0),
                placeholder,
                1
            )
            placeholder_idx += 1

        return modified_content, images

    def restore_images(
        self,
        content: str,
        images: List[ImageInfo],
        translate_alt: bool = False
    ) -> str:
        """
        Restore images from placeholders.

        Args:
            content: Content with placeholders
            images: List of original images
            translate_alt: Whether alt text was translated

        Returns:
            Content with restored images
        """
        modified = content

        for i, image in enumerate(images):
            if image.jsx:
                placeholder = f"__JSX_IMG_{i}__"
            else:
                placeholder = f"__IMAGE_{i}__"

            modified = modified.replace(placeholder, image.full_match)

        return modified

    def translate_alt_texts(
        self,
        images: List[ImageInfo],
        translations: Dict[str, str]
    ) -> List[ImageInfo]:
        """
        Update image alt texts with translations.

        Args:
            images: List of images
            translations: Dict mapping original alt to translated alt

        Returns:
            List of images with updated alt texts
        """
        updated = []

        for image in images:
            if image.alt_text in translations:
                new_alt = translations[image.alt_text]
                if image.jsx:
                    new_match = image.full_match.replace(
                        f'alt="{image.alt_text}"',
                        f'alt="{new_alt}"'
                    )
                else:
                    new_match = f'![{new_alt}]({image.url})'

                updated.append(ImageInfo(
                    alt_text=new_alt,
                    url=image.url,
                    full_match=new_match,
                    jsx=image.jsx
                ))
            else:
                updated.append(image)

        return updated
