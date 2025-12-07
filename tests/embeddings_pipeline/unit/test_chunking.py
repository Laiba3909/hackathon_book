# tests/embeddings_pipeline/unit/test_chunking.py

import unittest
from src.embeddings_pipeline.utils.chunking import TextChunker

class TestTextChunker(unittest.TestCase):
    def test_chunk_section_content_simple(self):
        chunker = TextChunker(max_chunk_size=10, overlap=0)
        content = "This is a short sentence. This is another one. And a third one."
        chunks = chunker.chunk_section_content(content)
        # Expected: ["This is a short sentence.", "This is another one.", "And a third one."]
        # With max_chunk_size=10, overlap=0, each sentence might be one chunk, or broken down more.
        # Let's adjust expected based on simple word split and max_chunk_size
        self.assertEqual(len(chunks), 10) # 3 sentences, but each is more than 10 words, so it will split.
        # The exact split points are implementation dependent based on word count.
        # Let's test for a specific content and chunk size
        
        chunker_small = TextChunker(max_chunk_size=3, overlap=0)
        content_small = "one two three four five six"
        chunks_small = chunker_small.chunk_section_content(content_small)
        self.assertEqual(chunks_small, ["one two three", "four five six"])

        chunker_overlap = TextChunker(max_chunk_size=5, overlap=2)
        content_overlap = "A B C D E F G H I J K L" # 12 words
        chunks_overlap = chunker_overlap.chunk_section_content(content_overlap)
        # Expected: ['A B C D E', 'D E F G H', 'G H I J K', 'J K L']
        self.assertEqual(chunks_overlap, ['A B C D E', 'D E F G H', 'G H I J K', 'J K L'])

    def test_chunk_section_content_paragraphs(self):
        chunker = TextChunker(max_chunk_size=15, overlap=0)
        content = "First paragraph with some words. It is fairly long.\n\nSecond paragraph here. Shorter."
        chunks = chunker.chunk_section_content(content)
        # Expect paragraphs to be respected first
        self.assertEqual(len(chunks), 2) # The first paragraph will be split due to max_chunk_size, then the second.
        # Let's refine based on paragraph handling
        # Para 1: "First paragraph with some words. It is fairly long." (10 words)
        # Para 2: "Second paragraph here. Shorter." (5 words)
        # With max_chunk_size = 15, no split should happen for these specific paras.
        # The implementation splits by paragraph first, then by word count within each.
        self.assertEqual(chunks, ["First paragraph with some words. It is fairly long.", "Second paragraph here. Shorter."])

    def test_chunk_empty_content(self):
        chunker = TextChunker()
        chunks = chunker.chunk_section_content("")
        self.assertEqual(chunks, [])
        chunks_whitespace = chunker.chunk_section_content("   \n\n  ")
        self.assertEqual(chunks_whitespace, [])

    def test_chunk_document_sections_with_metadata(self):
        chunker = TextChunker(max_chunk_size=10, overlap=0)
        dummy_sections = [
            {
                "level": 1,
                "title": "Introduction",
                "cid": "cid_intro",
                "content": "This is the introduction. It has some text. More text is here."
            },
            {
                "level": 2,
                "title": "Conclusion",
                "cid": "cid_conclusion",
                "content": "The conclusion wraps things up."
            }
        ]
        chunks_with_metadata = chunker.chunk_document_sections(dummy_sections)
        self.assertEqual(len(chunks_with_metadata), 2) # intro should split, conclusion is one
        # "This is the introduction. It has some text." (9 words) - 1 chunk
        # "More text is here." (4 words) - 1 chunk
        # "The conclusion wraps things up." (5 words) - 1 chunk
        # So total 3 chunks.
        
        self.assertEqual(chunks_with_metadata[0]['metadata']['section_title'], "Introduction")
        self.assertEqual(chunks_with_metadata[0]['metadata']['cid'], "cid_intro")
        self.assertEqual(chunks_with_metadata[0]['metadata']['chunk_index'], 0)
        self.assertEqual(chunks_with_metadata[1]['metadata']['section_title'], "Introduction")
        self.assertEqual(chunks_with_metadata[1]['metadata']['cid'], "cid_intro")
        self.assertEqual(chunks_with_metadata[1]['metadata']['chunk_index'], 1)
        self.assertEqual(chunks_with_metadata[2]['metadata']['section_title'], "Conclusion")
        self.assertEqual(chunks_with_metadata[2]['metadata']['cid'], "cid_conclusion")
        self.assertEqual(chunks_with_metadata[2]['metadata']['chunk_index'], 0)


if __name__ == '__main__':
    unittest.main()
