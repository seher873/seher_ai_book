import { Chapter } from '../../data-model'; // Assuming we have a data model

// Validation rules and utilities for Module 4 content

/**
 * Validates a chapter object according to Module 4 requirements
 * @param {Object} chapter - The chapter to validate
 * @returns {Object} Validation result with errors and warnings
 */
export function validateChapter(chapter) {
  const result = {
    isValid: true,
    errors: [],
    warnings: [],
    info: []
  };

  // Validate required fields
  if (!chapter.title || chapter.title.trim() === '') {
    result.errors.push({
      field: 'title',
      message: 'Chapter title is required',
      severity: 'error'
    });
    result.isValid = false;
  }

  if (!chapter.content || chapter.content.trim() === '') {
    result.errors.push({
      field: 'content',
      message: 'Chapter content is required',
      severity: 'error'
    });
    result.isValid = false;
  }

  // Validate learning objectives
  if (!chapter.learningObjectives || chapter.learningObjectives.length === 0) {
    result.warnings.push({
      field: 'learningObjectives',
      message: 'No learning objectives defined',
      severity: 'warning'
    });
  } else if (chapter.learningObjectives && chapter.learningObjectives.length > 0) {
    chapter.learningObjectives.forEach((objective, index) => {
      if (typeof objective !== 'string' || objective.trim() === '') {
        result.errors.push({
          field: `learningObjectives[${index}]`,
          message: `Learning objective ${index + 1} must be a non-empty string`,
          severity: 'error'
        });
        result.isValid = false;
      } else if (objective.length > 200) {
        result.warnings.push({
          field: `learningObjectives[${index}]`,
          message: `Learning objective ${index + 1} is quite long, consider shortening it`,
          severity: 'warning'
        });
      }
    });
  }

  // Validate sections
  if (chapter.sections && chapter.sections.length > 0) {
    chapter.sections.forEach((section, index) => {
      if (!section.title) {
        result.errors.push({
          field: `sections[${index}].title`,
          message: `Section ${index + 1} must have a title`,
          severity: 'error'
        });
        result.isValid = false;
      }

      if (!section.content) {
        result.errors.push({
          field: `sections[${index}].content`,
          message: `Section ${index + 1} must have content`,
          severity: 'error'
        });
        result.isValid = false;
      }

      if (!section.type) {
        result.errors.push({
          field: `sections[${index}].type`,
          message: `Section ${index + 1} must have a type`,
          severity: 'error'
        });
        result.isValid = false;
      } else if (!isValidSectionType(section.type)) {
        result.errors.push({
          field: `sections[${index}].type`,
          message: `Section ${index + 1} has an invalid type: ${section.type}`,
          severity: 'error'
        });
        result.isValid = false;
      }
    });
  }

  // Validate diagram placeholders
  if (chapter.diagramPlaceholders && chapter.diagramPlaceholders.length > 0) {
    chapter.diagramPlaceholders.forEach((diagram, index) => {
      if (!diagram.title) {
        result.errors.push({
          field: `diagramPlaceholders[${index}].title`,
          message: `Diagram ${index + 1} must have a title`,
          severity: 'error'
        });
        result.isValid = false;
      }

      if (!diagram.description) {
        result.errors.push({
          field: `diagramPlaceholders[${index}].description`,
          message: `Diagram ${index + 1} must have a description`,
          severity: 'error'
        });
        result.isValid = false;
      }
    });
  }

  // Validate code examples
  if (chapter.codeExamples && chapter.codeExamples.length > 0) {
    chapter.codeExamples.forEach((example, index) => {
      if (!example.title) {
        result.errors.push({
          field: `codeExamples[${index}].title`,
          message: `Code example ${index + 1} must have a title`,
          severity: 'error'
        });
        result.isValid = false;
      }

      if (!example.language) {
        result.errors.push({
          field: `codeExamples[${index}].language`,
          message: `Code example ${index + 1} must have a language specified`,
          severity: 'error'
        });
        result.isValid = false;
      }

      if (!example.code) {
        result.errors.push({
          field: `codeExamples[${index}].code`,
          message: `Code example ${index + 1} must have code content`,
          severity: 'error'
        });
        result.isValid = false;
      }
    });
  }

  return result;
}

/**
 * Validates content according to Module 4 requirements
 * @param {string} content - The content to validate
 * @returns {Object} Validation result
 */
export function validateContent(content) {
  const result = {
    isValid: true,
    errors: [],
    warnings: [],
    metadata: {}
  };

  // Check content length
  if (content && content.length < 500) {
    result.warnings.push({
      message: 'Content seems short. Consider adding more details or examples.',
      severity: 'warning'
    });
  }

  // Check for basic Markdown structure
  if (content && !content.includes('# ')) {
    result.info.push({
      message: 'No main heading detected. Consider adding a main heading (H1) to structure your content.',
      severity: 'info'
    });
  }

  // Check for diagram placeholders
  const diagramPlaceholderRegex = /!\[.+\]\(.+\.png\)/g;
  const diagramPlaceholders = content.match(diagramPlaceholderRegex) || [];
  result.metadata.diagramPlaceholdersCount = diagramPlaceholders.length;

  if (diagramPlaceholders.length === 0) {
    result.warnings.push({
      message: 'No diagram placeholders detected. Module 4 content should include visual elements.',
      severity: 'warning'
    });
  }

  // Check for code blocks
  const codeBlockRegex = /```[\s\S]*?```/g;
  const codeBlocks = content.match(codeBlockRegex) || [];
  result.metadata.codeBlocksCount = codeBlocks.length;

  if (codeBlocks.length === 0) {
    result.warnings.push({
      message: 'No code examples detected. Consider adding code examples for better understanding.',
      severity: 'warning'
    });
  }

  return result;
}

/**
 * Validates a learning objective
 * @param {string} objective - The objective to validate
 * @returns {Object} Validation result
 */
export function validateLearningObjective(objective) {
  const result = {
    isValid: true,
    errors: [],
    warnings: []
  };

  if (typeof objective !== 'string' || objective.trim() === '') {
    result.errors.push({ message: 'Learning objective must be a non-empty string', severity: 'error' });
    result.isValid = false;
  } else if (objective.length > 200) {
    result.warnings.push({
      message: 'Learning objective is quite long, consider shortening it for clarity',
      severity: 'warning'
    });
  }

  return result;
}

/**
 * Validates a section object
 * @param {Object} section - The section to validate
 * @returns {Object} Validation result
 */
export function validateSection(section) {
  const result = {
    isValid: true,
    errors: []
  };

  if (!section.title) {
    result.errors.push({ message: 'Section must have a title', severity: 'error' });
    result.isValid = false;
  }

  if (!section.content) {
    result.errors.push({ message: 'Section must have content', severity: 'error' });
    result.isValid = false;
  }

  if (!section.type) {
    result.errors.push({ message: 'Section must have a type', severity: 'error' });
    result.isValid = false;
  } else if (!isValidSectionType(section.type)) {
    result.errors.push({ message: `Invalid section type: ${section.type}`, severity: 'error' });
    result.isValid = false;
  }

  return result;
}

/**
 * Validates a code example object
 * @param {Object} example - The code example to validate
 * @returns {Object} Validation result
 */
export function validateCodeExample(example) {
  const result = {
    isValid: true,
    errors: []
  };

  if (!example.title) {
    result.errors.push({ message: 'Code example must have a title', severity: 'error' });
    result.isValid = false;
  }

  if (!example.language) {
    result.errors.push({ message: 'Code example must have a language specified', severity: 'error' });
    result.isValid = false;
  }

  if (!example.code) {
    result.errors.push({ message: 'Code example must have code content', severity: 'error' });
    result.isValid = false;
  }

  return result;
}

/**
 * Validates a diagram placeholder object
 * @param {Object} diagram - The diagram placeholder to validate
 * @returns {Object} Validation result
 */
export function validateDiagramPlaceholder(diagram) {
  const result = {
    isValid: true,
    errors: []
  };

  if (!diagram.title) {
    result.errors.push({ message: 'Diagram placeholder must have a title', severity: 'error' });
    result.isValid = false;
  }

  if (!diagram.description) {
    result.errors.push({ message: 'Diagram placeholder must have a description', severity: 'error' });
    result.isValid = false;
  }

  return result;
}

/**
 * Helper function to check if a section type is valid
 * @param {string} type - The type to check
 * @returns {boolean} True if valid, false otherwise
 */
function isValidSectionType(type) {
  const validTypes = ['concept', 'tutorial', 'example', 'summary', 'workflow'];
  return validTypes.includes(type.toLowerCase());
}

/**
 * Runs a comprehensive validation on the entire module
 * @param {Object} module - The module data to validate
 * @returns {Object} Comprehensive validation result
 */
export function validateModule(module) {
  const result = {
    isValid: true,
    errors: [],
    warnings: [],
    chapterResults: {}
  };

  if (!module.chapters || module.chapters.length === 0) {
    result.errors.push({ message: 'Module must have at least one chapter', severity: 'error' });
    result.isValid = false;
  } else {
    module.chapters.forEach((chapter, index) => {
      const chapterResult = validateChapter(chapter);
      result.chapterResults[`chapter_${index}`] = chapterResult;
      
      if (!chapterResult.isValid) {
        result.isValid = false;
        result.errors.push(...chapterResult.errors.map(error => ({
          ...error,
          chapterIndex: index
        })));
        result.warnings.push(...chapterResult.warnings.map(warning => ({
          ...warning,
          chapterIndex: index
        })));
      }
    });
  }

  return result;
}

export default {
  validateChapter,
  validateContent,
  validateLearningObjective,
  validateSection,
  validateCodeExample,
  validateDiagramPlaceholder,
  validateModule
};