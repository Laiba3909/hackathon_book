# src/embeddings_pipeline/utils/text_cleaner.py

import re
from typing import List

class TextCleaner:
    def __init__(self):
        pass

    def clean_text(self, text: str, strip_markdown: bool = True, remove_special_chars: bool = True, to_lowercase: bool = False) -> str:
        """
        Applies various normalization steps to a given text.
        :param text: The input string to clean.
        :param strip_markdown: If True, removes Markdown syntax.
        :param remove_special_chars: If True, removes non-alphanumeric/whitespace characters.
        :param to_lowercase: If True, converts text to lowercase.
        :return: The cleaned string.
        """
        if strip_markdown:
            text = self._strip_markdown(text)
        
        # Standardize whitespace
        text = re.sub(r'\s+', ' ', text).strip()

        if remove_special_chars:
            # Keep alphanumeric, spaces, and basic punctuation that might be semantically important (.,!?)
            text = re.sub(r'[^a-zA-Z0-9\s.,!?]', '', text)
        
        if to_lowercase:
            text = text.lower()
            
        return text

    def _strip_markdown(self, text: str) -> str:
        """
        Removes common Markdown syntax from text.
        """
        # Remove images and links (retain link text if available)
        text = re.sub(r'!\\\[(.*?)\\\]\(.*?\\)', r'\1', text) # Images
        text = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', text)  # Links

        # Remove headings
        text = re.sub(r'#{1,6}\s*(.*)', r'\1', text)

        # Remove bold, italic, strikethrough
        text = re.sub(r'(\*\*|__)(.*?)\1', r'\2', text) # Bold
        text = re.sub(r'(\*|_)(.*?)\1', r'\2', text)   # Italic
        text = re.sub(r'~~(.*?)~~', r'\1', text)       # Strikethrough

        # Remove blockquotes
        text = re.sub(r'^>\s*(.*)', r'\1', text, flags=re.MULTILINE)

        # Remove list markers
        text = re.sub(r'^\s*[-*+]\s+', '', text, flags=re.MULTILINE)
        text = re.sub(r'^\s*\d+\\.\s+', '', text, flags=re.MULTILINE)

        # Remove horizontal rules
        text = re.sub(r'^-{3,}\s*$', '', text, flags=re.MULTILINE)

        # Remove code blocks and inline code
        text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)
        text = re.sub(r'`(.*?)`', r'\1', text)
        
        return text

# Example usage (for testing)
if __name__ == "__main__":
    cleaner = TextCleaner()
    
    markdown_text = """
---
title: \"Test Document\"
---

# My Great Document

This is some **bold text** and *italic text*.
Here's a [link to Google](https://www.google.com).

## A Subheading

- List item 1
- List item 2

```python
print(\"Hello, World!\")
```

> This is a blockquote.

Another paragraph with `inline code`. And an image ![alt text](image.png).
"""
    
    cleaned_text = cleaner.clean_text(markdown_text, strip_markdown=True, remove_special_chars=True, to_lowercase=False)
    print("Cleaned Text (Markdown Stripped, Special Chars Removed):\n", cleaned_text)

    cleaned_text_lower = cleaner.clean_text(markdown_text, strip_markdown=True, remove_special_chars=True, to_lowercase=True)
    print("\nCleaned Text (Lowercase):\n", cleaned_text_lower)
