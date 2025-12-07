# src/embeddings_pipeline/utils/chunking.py

import re
from typing import List, Dict

class TextChunker:
    def __init__(self, max_chunk_size: int = 500, overlap: int = 50):
        """
        Initializes the TextChunker.
        :param max_chunk_size: Maximum desired length of a text chunk (e.g., in words or characters).
        :param overlap: Number of words/characters to overlap between consecutive chunks.
        """
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap

    def chunk_section_content(self, section_content: str) -> List[str]:
        """
        Chunks the content of a single section into smaller, semantically coherent pieces.
        Prioritizes paragraph breaks.
        """
        if not section_content.strip():
            return []

        # Split by paragraphs first
        paragraphs = [p.strip() for p in re.split(r'\n\s*\n', section_content) if p.strip()]
        
        chunks = []
        current_chunk_tokens = []
        current_chunk_length = 0

        for para in paragraphs:
            para_tokens = para.split() # Simple split by space, can be improved
            para_length = len(para_tokens)

            if current_chunk_length + para_length <= self.max_chunk_size:
                current_chunk_tokens.extend(para_tokens)
                current_chunk_length += para_length
            else:
                if current_chunk_tokens: # Save current chunk if it has content
                    chunks.append(" ".join(current_chunk_tokens))
                
                # Start new chunk with overlap
                overlap_tokens = current_chunk_tokens[-self.overlap:] if self.overlap > 0 else []
                current_chunk_tokens = overlap_tokens + para_tokens
                current_chunk_length = len(current_chunk_tokens)
                
                # If a single paragraph is larger than max_chunk_size, it needs to be split
                while current_chunk_length > self.max_chunk_size:
                    split_point = self.max_chunk_size - self.overlap
                    if split_point <= 0: # Avoid infinite loop for very small max_chunk_size with overlap
                        split_point = self.max_chunk_size // 2 if self.max_chunk_size > 1 else 1

                    chunks.append(" ".join(current_chunk_tokens[:split_point]))
                    current_chunk_tokens = current_chunk_tokens[split_point - self.overlap:]
                    current_chunk_length = len(current_chunk_tokens)
                
        if current_chunk_tokens:
            chunks.append(" ".join(current_chunk_tokens))
            
        return chunks

    def chunk_document_sections(self, sections: List[Dict]) -> List[Dict]:
        """
        Processes a list of sections (from MarkdownParser) and chunks their content.
        Adds metadata to each chunk for context.
        """
        all_chunks = []
        for section in sections:
            section_chunks = self.chunk_section_content(section['content'])
            for i, chunk_text in enumerate(section_chunks):
                chunk_metadata = {
                    "doc_id": section.get('doc_id'), # Assuming doc_id will be added by the pipeline
                    "filepath": section.get('filepath'), # Assuming filepath will be added by the pipeline
                    "chapter_label": section.get('chapter_label'), # Assuming chapter_label will be added by the pipeline
                    "section_title": section['title'],
                    "cid": section['cid'],
                    "heading_level": section['level'],
                    "chunk_index": i,
                    "total_chunks_in_section": len(section_chunks)
                }
                all_chunks.append({"text": chunk_text, "metadata": chunk_metadata})
        return all_chunks

# Example usage (for testing)
if __name__ == "__main__":

    # Dummy section for demonstration
    dummy_sections = [
        {
            "level": 1,
            "title": "Introduction to Robotics",
            "cid": "cid_intro_robotics",
            "content": "Robotics is an interdisciplinary field that integrates computer science and engineering. Robotics involves the design, construction, operation, and use of robots. The goal of robotics is to design machines that can help and assist humans.\n\nRobotics combines with AI to create intelligent systems. These systems can perceive, process, and act upon their environment autonomously. The future of robotics is exciting."
        },
        {
            "level": 2,
            "title": "History of Robots",
            "cid": "cid_history_robots",
            "content": "The concept of automated machines dates back to antiquity. Early automatons were often mythological or used for entertainment. The term 'robot' was coined by Karel Čapek in his 1920 play R.U.R.\n\nModern robotics began to take shape in the 20th century with the advent of programmable machines and advanced electronics. Industrial robots revolutionized manufacturing processes."
        }
    ]

    chunker = TextChunker(max_chunk_size=30, overlap=10)
    chunks = chunker.chunk_document_sections(dummy_sections)

    for i, chunk in enumerate(chunks):
        print(f"Chunk {i+1}:")
        print(f"  Metadata: {chunk['metadata']}")
        print(f"  Text: {chunk['text']}")
        print("-" * 30)
