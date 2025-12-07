# tests/embeddings_pipeline/unit/test_text_cleaner.py

import unittest
from src.embeddings_pipeline.utils.text_cleaner import TextCleaner

class TestTextCleaner(unittest.TestCase):
    def setUp(self):
        self.cleaner = TextCleaner()

    def test_strip_markdown(self):
        markdown_text = """
# Heading 1

## Heading 2

This is **bold** and *italic*.
[Link text](http://example.com) and ![Image alt](http://example.com/img.png).
`inline code` and a `second inline code`.

```python
print("Hello")
```

> Blockquote content.

- List item 1
- List item 2

---
"""
        expected_cleaned = "Heading 1 Heading 2 This is bold and italic. Link text and Image alt. inline code and a second inline code. Blockquote content. List item 1 List item 2"
        cleaned_text = self.cleaner.clean_text(markdown_text, strip_markdown=True, remove_special_chars=False, to_lowercase=False)
        # We replace multiple spaces with single space in clean_text
        self.assertEqual(cleaned_text.strip(), expected_cleaned)
        
    def test_remove_special_chars(self):
        text = "Hello, World! This is a test. @#$!%^&*()_+"
        expected_cleaned = "Hello World This is a test."
        cleaned_text = self.cleaner.clean_text(text, strip_markdown=False, remove_special_chars=True, to_lowercase=False)
        self.assertEqual(cleaned_text.strip(), expected_cleaned)

    def test_to_lowercase(self):
        text = "HELLO World Test"
        expected_cleaned = "hello world test"
        cleaned_text = self.cleaner.clean_text(text, strip_markdown=False, remove_special_chars=False, to_lowercase=True)
        self.assertEqual(cleaned_text.strip(), expected_cleaned)

    def test_combined_cleaning(self):
        markdown_text = """
# Test Doc!

This is **important** text.
It has a [link](url) and some special chars: @#$.
"""
        expected_cleaned = "test doc this is important text it has a link and some special chars"
        cleaned_text = self.cleaner.clean_text(markdown_text, strip_markdown=True, remove_special_chars=True, to_lowercase=True)
        self.assertEqual(cleaned_text.strip(), expected_cleaned)

    def test_code_block_removal(self):
        text_with_code = """
Some text.

```
def func():
    return True
```

More text.
"""
        expected_cleaned = "Some text. More text."
        cleaned_text = self.cleaner.clean_text(text_with_code, strip_markdown=True, remove_special_chars=False, to_lowercase=False)
        self.assertEqual(cleaned_text.strip(), expected_cleaned)

if __name__ == '__main__':
    unittest.main()
