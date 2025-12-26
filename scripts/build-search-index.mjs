// scripts/build-search-index.mjs
import { promises as fs } from 'fs';
import path from 'path';
import { glob } from 'glob';
import matter from 'gray-matter';
import { remark } from 'remark';
import strip from 'strip-markdown';
import FlexSearch from 'flexsearch';

// This is where the generated index will be saved
const outputDir = path.join(process.cwd(), 'static');
const outputFile = path.join(outputDir, 'search-index.json');
const docsDir = path.join(process.cwd(), 'docs');

// A function to convert markdown to plain text
async function markdownToText(markdown) {
  const result = await remark().use(strip).process(markdown);
  return result.toString();
}

// A function to split a document into sections based on headings
function splitIntoSections(content, filepath) {
    // A document is at least one section
    const sections = [];
    // Split by Markdown headings (level 2 and 3)
    const parts = content.split(/(?=^##\s|^###\s)/m);
    
    let currentTitle = path.basename(filepath, path.extname(filepath));

    for (const part of parts) {
        if (!part.trim()) continue;

        const lines = part.split('\n');
        const headingMatch = lines[0].match(/^(##|###)\s+(.*)/);
        let title = currentTitle;
        let content = part;

        if (headingMatch) {
            title = headingMatch[2];
            content = lines.slice(1).join('\n');
        }

        sections.push({
            title: title.trim(),
            content: content.trim(),
            filepath: filepath.replace(process.cwd(),'').replace(/\\/g, '/') // make path relative and clean
        });
    }
    return sections;
}


async function buildSearchIndex() {
  console.log('Starting to build search index...');
  
  // Using a Document index in FlexSearch
  // https://github.com/nextapps-de/flexsearch#document-index
  const index = new FlexSearch.Document({
    document: {
      id: 'id',
      index: ['title', 'content'],
      store: ['title', 'content', 'filepath'],
    },
    tokenize: 'forward',
  });

  const files = await glob('**/*.{md,mdx}', { cwd: docsDir });
  let docId = 0;

  for (const file of files) {
    const filepath = path.join(docsDir, file);
    console.log(`Processing: ${filepath}`);
    
    const fileContent = await fs.readFile(filepath, 'utf8');
    const { content: markdownContent } = matter(fileContent); // Extract content, ignoring frontmatter
    
    // Convert the entire doc to plain text to split into sections
    const plainTextContent = await markdownToText(markdownContent);
    const sections = splitIntoSections(plainTextContent, filepath);

    for (const section of sections) {
        if (section.content) {
            index.add({
                id: docId++,
                title: section.title,
                content: section.content,
                // We store the full details in the store for later retrieval
                filepath: section.filepath, 
            });
        }
    }
  }

  console.log(`Indexed ${docId} sections from ${files.length} files.`);

  // Export the index. We will export each part of the index separately.
  const exportedIndex = {};
  index.export((key, data) => {
    // For some reason, data can be null in some cases, so we check for it.
    if (data) {
      exportedIndex[key] = data;
    }
  });

  // Ensure output directory exists
  await fs.mkdir(outputDir, { recursive: true });
  // Write the exported index to the file
  await fs.writeFile(outputFile, JSON.stringify(exportedIndex));

  console.log(`Search index successfully built at: ${outputFile}`);
}

buildSearchIndex().catch(console.error);
