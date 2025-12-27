// Mock API service for content creation interface
// In a real implementation, this would communicate with a backend API

const CONTENT_BASE_PATH = '/docs/module-4';
const CHAPTER_FILES = [
  { id: 'index', path: `${CONTENT_BASE_PATH}/index.md` },
  { id: '01-intro-vla', path: `${CONTENT_BASE_PATH}/01-intro-vla.md` },
  { id: '02-speech-to-text', path: `${CONTENT_BASE_PATH}/02-speech-to-text.md` },
  { id: '03-task-decomposition', path: `${CONTENT_BASE_PATH}/03-task-decomposition.md` },
  { id: '04-multimodal-perception', path: `${CONTENT_BASE_PATH}/04-multimodal-perception.md` },
  { id: '05-ros2-planning-execution', path: `${CONTENT_BASE_PATH}/05-ros2-planning-execution.md` },
  { id: '06-capstone-humanoid', path: `${CONTENT_BASE_PATH}/06-capstone-humanoid.md` }
];

// Simulated content storage
let contentStore = {
  'index': {
    id: 'index',
    title: 'Module 4 - LLMs + Robotics: Voice-to-Action Systems',
    content: '# Module 4: LLMs + Robotics: Voice-to-Action Systems\n\n## Overview\n\nWelcome to Module 4...',
    learningObjectives: [
      'Understand Vision-Language-Action (VLA) models and their applications',
      'Convert voice commands to text for robotic processing',
      'Decompose natural language commands into executable robotic actions'
    ],
    sections: [],
    diagramPlaceholders: [{ title: 'Module Overview', description: 'Diagram showing the module structure' }],
    codeExamples: [],
    createdAt: '2025-12-14T00:00:00Z',
    updatedAt: '2025-12-14T00:00:00Z'
  }
};

/**
 * Get all available chapters for Module 4
 * @returns {Promise<Array>} List of chapter metadata
 */
export async function getAllChapters() {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve(CHAPTER_FILES.map(file => ({
        ...file,
        title: contentStore[file.id]?.title || file.id,
        status: contentStore[file.id] ? 'completed' : 'not-started',
        wordCount: contentStore[file.id] ? contentStore[file.id].content.split(/\s+/).length : 0
      })));
    }, 200);
  });
}

/**
 * Get a specific chapter by ID
 * @param {string} chapterId - The ID of the chapter to retrieve
 * @returns {Promise<Object>} Chapter content and metadata
 */
export async function getChapter(chapterId) {
  return new Promise((resolve, reject) => {
    setTimeout(() => {
      const chapter = contentStore[chapterId];
      if (chapter) {
        resolve(chapter);
      } else {
        // Return a default structure for new chapters
        resolve({
          id: chapterId,
          title: '',
          content: '',
          learningObjectives: [],
          sections: [],
          diagramPlaceholders: [],
          codeExamples: [],
          createdAt: new Date().toISOString(),
          updatedAt: new Date().toISOString()
        });
      }
    }, 300);
  });
}

/**
 * Save a chapter
 * @param {string} chapterId - The ID of the chapter to save
 * @param {Object} chapterData - The chapter content and metadata
 * @returns {Promise<Object>} Result of the save operation
 */
export async function saveChapter(chapterId, chapterData) {
  return new Promise((resolve, reject) => {
    setTimeout(() => {
      try {
        const now = new Date().toISOString();
        contentStore[chapterId] = {
          ...chapterData,
          id: chapterId,
          updatedAt: now
        };
        
        if (!contentStore[chapterId].createdAt) {
          contentStore[chapterId].createdAt = now;
        }
        
        resolve({
          success: true,
          message: 'Chapter saved successfully',
          chapterId: chapterId,
          filePath: `${CONTENT_BASE_PATH}/${chapterId}.md`
        });
      } catch (error) {
        reject({
          success: false,
          message: 'Failed to save chapter',
          error: error.message
        });
      }
    }, 400);
  });
}

/**
 * Create a new module structure
 * @param {Object} moduleData - The module structure to create
 * @returns {Promise<Object>} Result of the creation operation
 */
export async function createModuleStructure(moduleData) {
  return new Promise((resolve) => {
    setTimeout(() => {
      // In a real implementation, this would create the necessary files
      resolve({
        status: 'success',
        message: 'Module structure created successfully',
        filesGenerated: [
          `${CONTENT_BASE_PATH}/index.md`,
          `${CONTENT_BASE_PATH}/01-intro-vla.md`,
          `${CONTENT_BASE_PATH}/02-speech-to-text.md`,
          `${CONTENT_BASE_PATH}/03-task-decomposition.md`,
          `${CONTENT_BASE_PATH}/04-multimodal-perception.md`,
          `${CONTENT_BASE_PATH}/05-ros2-planning-execution.md`,
          `${CONTENT_BASE_PATH}/06-capstone-humanoid.md`
        ]
      });
    }, 500);
  });
}

/**
 * Update a chapter
 * @param {string} chapterId - The ID of the chapter to update
 * @param {Object} chapterData - The updated chapter content and metadata
 * @returns {Promise<Object>} Result of the update operation
 */
export async function updateChapter(chapterId, chapterData) {
  return saveChapter(chapterId, chapterData);
}

/**
 * Delete a chapter
 * @param {string} chapterId - The ID of the chapter to delete
 * @returns {Promise<Object>} Result of the deletion operation
 */
export async function deleteChapter(chapterId) {
  return new Promise((resolve, reject) => {
    setTimeout(() => {
      try {
        if (contentStore[chapterId]) {
          delete contentStore[chapterId];
          resolve({
            success: true,
            message: 'Chapter deleted successfully',
            chapterId: chapterId
          });
        } else {
          reject({
            success: false,
            message: 'Chapter not found',
            chapterId: chapterId
          });
        }
      } catch (error) {
        reject({
          success: false,
          message: 'Failed to delete chapter',
          error: error.message
        });
      }
    }, 300);
  });
}

/**
 * Get content creation status and statistics
 * @returns {Promise<Object>} Content creation status and statistics
 */
export async function getContentStatus() {
  return new Promise((resolve) => {
    setTimeout(() => {
      const completedChapters = Object.keys(contentStore).length;
      
      resolve({
        totalChapters: 7,
        completedChapters: completedChapters,
        lastUpdated: new Date().toISOString(),
        estimatedCompletionTime: '2-3 weeks',
        moduleProgress: Math.round((completedChapters / 7) * 100) + '%'
      });
    }, 200);
  });
}

export default {
  getAllChapters,
  getChapter,
  saveChapter,
  createModuleStructure,
  updateChapter,
  deleteChapter,
  getContentStatus
};