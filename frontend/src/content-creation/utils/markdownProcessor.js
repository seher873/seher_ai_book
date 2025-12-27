/**
 * Utility functions for processing and manipulating Markdown content
 * for Module 4 content creation
 */

/**
 * Creates the frontmatter for a Markdown file with proper metadata
 * @param {string} title - The title for the document
 * @param {string} [sidebarLabel] - Optional sidebar label
 * @param {string} [description] - Optional description
 * @param {Array<string>} [keywords] - Optional keywords
 * @returns {string} The frontmatter as a string
 */
export function createFrontmatter(title, sidebarLabel, description, keywords) {
  let frontmatter = '---\n';
  frontmatter += `title: ${title}\n`;
  
  if (sidebarLabel) {
    frontmatter += `sidebar_label: ${sidebarLabel}\n`;
  }
  
  if (description) {
    frontmatter += `description: ${description}\n`;
  }
  
  if (keywords && Array.isArray(keywords) && keywords.length > 0) {
    frontmatter += `keywords: [${keywords.join(', ')}]\n`;
  }
  
  frontmatter += '---\n\n';
  
  return frontmatter;
}

/**
 * Generates a Markdown file containing chapter content
 * @param {Object} chapterData - The chapter data to convert to Markdown
 * @returns {string} The complete Markdown content
 */
export function generateChapterMarkdown(chapterData) {
  let markdown = '';
  
  // Add frontmatter
  markdown += createFrontmatter(
    chapterData.title || 'Untitled Chapter',
    chapterData.sidebar_label || 'Chapter',
    chapterData.description,
    chapterData.keywords
  );
  
  // Add main title
  markdown += `# ${chapterData.title || 'Untitled Chapter'}\n\n`;
  
  // Add learning objectives if provided
  if (chapterData.learningObjectives && chapterData.learningObjectives.length > 0) {
    markdown += '## Learning Objectives\n\n';
    markdown += 'By the end of this chapter, you will be able to:\n';
    
    for (const objective of chapterData.learningObjectives) {
      if (objective.trim() !== '') {
        markdown += `- ${objective}\n`;
      }
    }
    
    markdown += '\n';
  }
  
  // Add main content
  if (chapterData.content) {
    markdown += chapterData.content;
    if (!chapterData.content.endsWith('\n')) {
      markdown += '\n';
    }
    markdown += '\n';
  }
  
  // Add sections if provided
  if (chapterData.sections && Array.isArray(chapterData.sections)) {
    for (const section of chapterData.sections) {
      if (section.title) {
        markdown += `### ${section.title}\n\n`;
      }
      
      if (section.content) {
        markdown += `${section.content}\n\n`;
      }
      
      // Add type information if it's relevant to the section
      if (section.type) {
        markdown += `<strong>Section Type:</strong> ${section.type}\n\n`;
      }
    }
  }
  
  // Add diagram placeholders
  if (chapterData.diagramPlaceholders && Array.isArray(chapterData.diagramPlaceholders)) {
    markdown += '## Diagrams\n\n';
    
    for (const diagram of chapterData.diagramPlaceholders) {
      if (diagram.title && diagram.description) {
        // Format: ![Description](path/to/image.png)
        markdown += `![${diagram.title}: ${diagram.description}](/img/module-4/${sanitizeFilename(diagram.title)}.png)\n\n`;
      }
    }
  }
  
  // Add code examples
  if (chapterData.codeExamples && Array.isArray(chapterData.codeExamples)) {
    for (const example of chapterData.codeExamples) {
      if (example.title) {
        markdown += `### ${example.title}\n\n`;
      }
      
      if (example.language && example.code) {
        markdown += `\`\`\`${example.language}\n`;
        markdown += `${example.code}\n`;
        markdown += '```\n\n';
      }
      
      if (example.explanation) {
        markdown += `${example.explanation}\n\n`;
      }
    }
  }
  
  // Add summary if provided
  if (chapterData.summary) {
    markdown += '## Summary\n\n';
    markdown += `${chapterData.summary}\n\n`;
  }
  
  // Add review questions if provided
  if (chapterData.reviewQuestions && Array.isArray(chapterData.reviewQuestions) && chapterData.reviewQuestions.length > 0) {
    markdown += '## Review Questions\n\n';
    
    for (let i = 0; i < chapterData.reviewQuestions.length; i++) {
      markdown += `${i + 1}. ${chapterData.reviewQuestions[i]}\n\n`;
    }
  }
  
  // Add next steps if provided
  if (chapterData.nextSteps) {
    markdown += '## Next Steps\n\n';
    markdown += `${chapterData.nextSteps}\n\n`;
  }
  
  return markdown;
}

/**
 * Sanitizes a string to be used as a filename
 * @param {string} str - The string to sanitize
 * @returns {string} The sanitized filename
 */
export function sanitizeFilename(str) {
  if (!str) return '';
  
  // Replace special characters with hyphens and convert to lowercase
  return str
    .toLowerCase()
    .replace(/[^\w\s-]/g, '') // Remove special characters
    .replace(/[\s_-]+/g, '-') // Replace spaces and underscores with hyphens
    .replace(/^-+|-+$/g, ''); // Remove leading/trailing hyphens
}

/**
 * Parses a Markdown string to extract information about the content
 * @param {string} markdownContent - The Markdown content to parse
 * @returns {Object} Parsed information about the content
 */
export function parseMarkdown(markdownContent) {
  const result = {
    frontmatter: {},
    title: '',
    h1: '',
    h2Count: 0,
    h3Count: 0,
    wordCount: 0,
    imageCount: 0,
    codeBlockCount: 0,
    diagramPlaceholders: [],
    mainContent: ''
  };
  
  // Extract frontmatter if present
  const frontmatterMatch = markdownContent.match(/^---\n([\s\S]*?)\n---\n/);
  if (frontmatterMatch) {
    const frontmatterStr = frontmatterMatch[1];
    result.frontmatter = parseFrontmatter(frontmatterStr);
  }
  
  // Extract main content (without frontmatter)
  const mainContentStart = frontmatterMatch ? frontmatterMatch[0].length : 0;
  result.mainContent = markdownContent.substring(mainContentStart);
  
  // Extract titles and count headings
  const lines = result.mainContent.split('\n');
  for (const line of lines) {
    if (line.startsWith('# ')) {
      result.h1 = line.substring(2).trim();
      result.title = result.frontmatter.title || result.h1;
    } else if (line.startsWith('## ')) {
      result.h2Count++;
    } else if (line.startsWith('### ')) {
      result.h3Count++;
    }
  }
  
  // Count words
  const words = result.mainContent.trim().split(/\s+/);
  result.wordCount = words.length > 0 ? words.length : 0;
  
  // Count images
  const imageRegex = /!\[.*?\]\(.*?\)/g;
  result.imageCount = (result.mainContent.match(imageRegex) || []).length;
  
  // Count code blocks
  const codeBlockRegex = /^```[\s\S]*?^```/gm;
  result.codeBlockCount = (result.mainContent.match(codeBlockRegex) || []).length;
  
  // Extract diagram placeholders
  const diagramRegex = /!\[(.*?)\]\((\/img\/module-4\/.*?)\)/g;
  let match;
  while ((match = diagramRegex.exec(result.mainContent)) !== null) {
    result.diagramPlaceholders.push({
      description: match[1],
      path: match[2]
    });
  }
  
  return result;
}

/**
 * Parses frontmatter from a string
 * @param {string} frontmatterStr - The frontmatter content as a string
 * @returns {Object} Parsed frontmatter as an object
 */
function parseFrontmatter(frontmatterStr) {
  const lines = frontmatterStr.split('\n');
  const result = {};
  
  let currentKey = '';
  let insideMultilineValue = false;
  let multilineValue = '';
  
  for (const line of lines) {
    if (insideMultilineValue) {
      if (line.trim() === '...' || line.endsWith(':') || line.startsWith('  ') === false) {
        // End of multiline value (if we encounter a new key)
        const trimmedLine = line.trim();
        if (trimmedLine && trimmedLine.endsWith(':')) {
          // Actually this is a new key ending our multiline value
          result[currentKey] = multilineValue.trim();
          insideMultilineValue = false;
          currentKey = trimmedLine.slice(0, -1); // Remove colon
        } else {
          multilineValue += '\n' + line;
          continue;
        }
      } else {
        multilineValue += '\n' + line;
        continue;
      }
    }
    
    const match = line.match(/^(\w+):\s*(.*)/);
    if (match) {
      currentKey = match[1];
      let value = match[2];
      
      if (value.startsWith('|')) {
        // Multiline value (commonmark)
        insideMultilineValue = true;
        multilineValue = value.substring(1).trim(); // Remove the pipe character
      } else if (value.startsWith('"') && value.endsWith('"')) {
        // Quoted value
        result[currentKey] = value.substring(1, value.length - 1);
      } else if (value.startsWith('[') && value.endsWith(']')) {
        // Array value
        result[currentKey] = value
          .substring(1, value.length - 1)
          .split(',')
          .map(item => item.trim().replace(/^["']|["']$/g, ''));
      } else if (value === 'true' || value === 'false') {
        // Boolean value
        result[currentKey] = value === 'true';
      } else {
        // Regular value
        if (value === '') {
          // Empty value, we'll check next lines for multiline
          insideMultilineValue = true;
          multilineValue = '';
        } else {
          result[currentKey] = value.trim();
        }
      }
    }
  }
  
  if (insideMultilineValue && currentKey) {
    result[currentKey] = multilineValue.trim();
  }
  
  return result;
}

/**
 * Validates the structure of a Markdown document for Module 4
 * @param {string} markdownContent - The Markdown content to validate
 * @returns {Object} Validation result with errors and warnings
 */
export function validateMarkdownStructure(markdownContent) {
  const result = {
    isValid: true,
    errors: [],
    warnings: []
  };
  
  const parsed = parseMarkdown(markdownContent);
  
  // Check for required frontmatter
  if (!parsed.frontmatter.title) {
    result.errors.push({
      message: 'Missing required "title" in frontmatter',
      severity: 'error'
    });
    result.isValid = false;
  }
  
  // Check for main heading
  if (!parsed.h1) {
    result.warnings.push({
      message: 'No main heading (H1) found in content',
      severity: 'warning'
    });
  }
  
  // Check for learning objectives
  if (!parsed.mainContent.includes('Learning Objectives') && !parsed.mainContent.includes('learning-objectives')) {
    result.warnings.push({
      message: 'No learning objectives section found',
      severity: 'warning'
    });
  }
  
  // Check for summary section
  if (!parsed.mainContent.toLowerCase().includes('summary')) {
    result.warnings.push({
      message: 'No summary section found',
      severity: 'warning'
    });
  }
  
  // Check for adequate content length
  if (parsed.wordCount < 500) {
    result.warnings.push({
      message: `Content appears short (${parsed.wordCount} words). Consider adding more detail.`,
      severity: 'warning'
    });
  }
  
  // Check for diagram placeholders
  if (parsed.diagramPlaceholders.length === 0) {
    result.warnings.push({
      message: 'No diagram placeholders found. Module 4 content should include visual elements.',
      severity: 'warning'
    });
  }
  
  // Check for code examples
  if (parsed.codeBlockCount === 0) {
    result.warnings.push({
      message: 'No code examples found. Consider including code blocks for better understanding.',
      severity: 'warning'
    });
  }
  
  return result;
}

/**
 * Formats a Markdown document for consistency
 * @param {string} markdownContent - The Markdown content to format
 * @returns {string} Formatted Markdown content
 */
export function formatMarkdown(markdownContent) {
  // Currently just returns the content as is, but can be expanded
  // to format the content according to style guidelines
  
  // Example formatting: ensure there's a blank line after frontmatter
  let formattedContent = markdownContent.replace(/(---\n)([\w])/g, '$1\n$2');
  
  // Ensure there's a newline at the end of the document
  if (!formattedContent.endsWith('\n')) {
    formattedContent += '\n';
  }
  
  return formattedContent;
}

export default {
  createFrontmatter,
  generateChapterMarkdown,
  sanitizeFilename,
  parseMarkdown,
  validateMarkdownStructure,
  formatMarkdown
};