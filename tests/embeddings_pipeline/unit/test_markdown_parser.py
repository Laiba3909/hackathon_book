# tests/embeddings_pipeline/unit/test_markdown_parser.py

import unittest
import os
from src.embeddings_pipeline.utils.markdown_parser import MarkdownParser

class TestMarkdownParser(unittest.TestCase):
    def setUp(self):
        self.parser = MarkdownParser(docs_manifest_path='../../.specify/docs-manifest.json')
        self.test_dir = os.path.join(os.path.dirname(__file__), 'temp_test_docs')
        os.makedirs(self.test_dir, exist_ok=True)

    def tearDown(self):
        if os.path.exists(self.test_dir):
            import shutil
            shutil.rmtree(self.test_dir)

    def _create_test_file(self, filename, content):
        filepath = os.path.join(self.test_dir, filename)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        return filepath

    def test_parse_simple_markdown(self):
        content = """
---
title: Test Doc
---

<!-- CID: a1b2c3d4e5f6 -->
# Main Title

This is some content under the main title.

## Subsection 1

Content for subsection 1.

<!-- CID: d4e5f6a1b2c3 -->
### Sub-subsection

More content.
"""
        filepath = self._create_test_file("simple.md", content)
        frontmatter, sections = self.parser.parse_markdown_file(filepath)

        self.assertIsNotNone(frontmatter)
        self.assertEqual(frontmatter.get('title'), 'Test Doc')
        self.assertEqual(len(sections), 3)

        self.assertEqual(sections[0]['level'], 1)
        self.assertEqual(sections[0]['title'], 'Main Title')
        self.assertEqual(sections[0]['cid'], 'a1b2c3d4e5f6')
        self.assertIn('This is some content', sections[0]['content'])

        self.assertEqual(sections[1]['level'], 2)
        self.assertEqual(sections[1]['title'], 'Subsection 1')
        self.assertIsNone(sections[1]['cid'])
        self.assertIn('Content for subsection 1', sections[1]['content'])

        self.assertEqual(sections[2]['level'], 3)
        self.assertEqual(sections[2]['title'], 'Sub-subsection')
        self.assertEqual(sections[2]['cid'], 'd4e5f6a1b2c3')
        self.assertIn('More content', sections[2]['content'])

    def test_parse_no_frontmatter(self):
        content = """
# No Frontmatter

Just content.
"""
        filepath = self._create_test_file("no_frontmatter.md", content)
        frontmatter, sections = self.parser.parse_markdown_file(filepath)
        self.assertEqual(frontmatter, {})
        self.assertEqual(len(sections), 1)
        self.assertEqual(sections[0]['title'], 'No Frontmatter')

    def test_empty_file(self):
        filepath = self._create_test_file("empty.md", "")
        frontmatter, sections = self.parser.parse_markdown_file(filepath)
        self.assertEqual(frontmatter, {})
        self.assertEqual(sections, [])

    def test_file_not_found(self):
        frontmatter, sections = self.parser.parse_markdown_file("non_existent_file.md")
        self.assertIsNone(frontmatter)
        self.assertEqual(sections, [])

    def test_content_after_last_heading(self):
        content = """
# Title
Content 1.
Content 2.
"""
        filepath = self._create_test_file("last_content.md", content)
        frontmatter, sections = self.parser.parse_markdown_file(filepath)
        self.assertEqual(len(sections), 1)
        self.assertEqual(sections[0]['title'], 'Title')
        self.assertIn('Content 1.', sections[0]['content'])
        self.assertIn('Content 2.', sections[0]['content'])

if __name__ == '__main__':
    unittest.main()
