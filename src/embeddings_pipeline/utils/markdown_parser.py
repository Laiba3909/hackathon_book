# src/embeddings_pipeline/utils/markdown_parser.py

import re
import os
import yaml
import json

class MarkdownParser:
    def __init__(self, docs_manifest_path: str = '../../.specify/docs-manifest.json'):
        self.docs_manifest_path = os.path.join(os.path.dirname(__file__), docs_manifest_path)
        self.docs_manifest = self._load_docs_manifest()

    def _load_docs_manifest(self):
        if os.path.exists(self.docs_manifest_path):
            with open(self.docs_manifest_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return []

    def parse_markdown_file(self, filepath: str):
        """
        Parses a Markdown file to extract frontmatter and content sections with headings.
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
        except FileNotFoundError:
            return None, []

        frontmatter, markdown_content = self._extract_frontmatter(content)
        sections = self._extract_sections(markdown_content)
        
        return frontmatter, sections

    def _extract_frontmatter(self, content: str):
        """
        Extracts YAML frontmatter from the beginning of the Markdown content.
        """
        match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)
        if match:
            frontmatter_str = match.group(1)
            try:
                frontmatter = yaml.safe_load(frontmatter_str)
            except yaml.YAMLError:
                frontmatter = {}
            remaining_content = content[match.end():]
            return frontmatter, remaining_content
        return {}, content

    def _extract_sections(self, markdown_content: str):
        """
        Extracts sections based on headings (H1-H6) and their content.
        Assumes CIDs are already present as HTML comments directly before headings.
        """
        sections = []
        # Regex to find headings and optional preceding CID comments
        # Group 1: CID comment (optional)
        # Group 2: CID value
        # Group 3: Heading characters (e.g., #, ##)
        # Group 4: Heading title
        pattern = re.compile(r'(<!--\s*CID:\s*([a-f0-9]+)\s*-->\s*\n)?(#{1,6}\s+.*)', re.MULTILINE)
        
        # Split content by headings
        matches = list(pattern.finditer(markdown_content))
        
        start_idx = 0
        current_section = None

        for i, match in enumerate(matches):
            # If there's a previous section, its content ends here
            if current_section:
                current_section_content = markdown_content[start_idx:match.start()].strip()
                current_section['content'] = current_section_content
                sections.append(current_section)

            # Start a new section
            cid = match.group(2) if match.group(1) else None
            heading_line = match.group(3)
            level = len(heading_line.split(' ')[0])
            title = heading_line[level:].strip()

            current_section = {
                "level": level,
                "title": title,
                "cid": cid,
                "content": "" # Will be filled by next iteration or after loop
            }
            start_idx = match.end()
        
        # Add the last section's content
        if current_section:
            current_section_content = markdown_content[start_idx:].strip()
            current_section['content'] = current_section_content
            sections.append(current_section)

        return sections

# Example usage (for testing)
if __name__ == "__main__":
    # Assuming this script is run from src/embeddings_pipeline/
    # And the docs are in docbook/docs/
    base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'docs'))
    filepath = os.path.join(base_path, 'intro.md') # Example file
    
    parser = MarkdownParser()
    frontmatter, sections = parser.parse_markdown_file(filepath)

    print("Frontmatter:", frontmatter)
    for section in sections:
        print(f"--- Section ---")
        print(f"Level: {section['level']}")
        print(f"Title: {section['title']}")
        print(f"CID: {section['cid']}")
        # print(f"Content:\n{section['content'][:200]}...") # Print first 200 chars of content
        print("-" * 20)
