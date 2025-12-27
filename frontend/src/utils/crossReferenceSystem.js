/**
 * Cross-referencing system for linking between content components
 * This utility helps link assessments, labs, glossary terms, and appendices
 */

/**
 * Function to find related glossary terms for a given content
 * @param {string} content - The content to analyze for glossary terms
 * @returns {Array} Array of related glossary term IDs
 */
export function findRelatedGlossaryTerms(content) {
  // This would typically connect to a glossary index/database
  // For now, we'll implement a simple keyword matching system
  const glossaryTerms = [
    { id: 'algorithm', keywords: ['algorithm', 'algorithms'] },
    { id: 'neural-network', keywords: ['neural network', 'neural networks', 'deep learning'] },
    { id: 'machine-learning', keywords: ['machine learning', 'ml'] },
    { id: 'ai', keywords: ['artificial intelligence', 'AI', 'ai'] },
    { id: 'data-science', keywords: ['data science', 'data scientist'] },
    // Add more terms as needed
  ];

  const foundTerms = [];
  const contentLower = content.toLowerCase();

  glossaryTerms.forEach(term => {
    term.keywords.forEach(keyword => {
      if (contentLower.includes(keyword.toLowerCase()) && !foundTerms.includes(term.id)) {
        foundTerms.push(term.id);
      }
    });
  });

  return foundTerms;
}

/**
 * Function to find related assessments for a given module
 * @param {string} moduleId - The module ID to find related assessments for
 * @returns {Array} Array of related assessment IDs
 */
export function findRelatedAssessments(moduleId) {
  // This would typically connect to an assessment index/database
  // For now, we'll implement a simple matching system
  const assessmentMap = {
    'module-1': ['introduction-quiz'],
    'module-2': ['digital-twin-quiz'],
    'module-3': ['simulation-quiz'],
    'module-4': ['vlm-quiz'],
    // Add more mappings as needed
  };

  return assessmentMap[moduleId] || [];
}

/**
 * Function to find related labs for a given module or topic
 * @param {string} moduleId - The module ID to find related labs for
 * @returns {Array} Array of related lab IDs
 */
export function findRelatedLabs(moduleId) {
  // This would typically connect to a lab index/database
  // For now, we'll implement a simple matching system
  const labMap = {
    'module-1': ['lab1-1-ros2-installation', 'lab1-2-first-nodes'],
    'module-2': ['lab2-1-gazebo-basics', 'lab2-2-unity-robotics'],
    'module-3': ['lab3-1-isaac-sim', 'lab3-2-synthetic-data'],
    'module-4': ['lab4-1-voice-command', 'lab4-2-vlm-integration'],
    // Add more mappings as needed
  };

  return labMap[moduleId] || [];
}

/**
 * Function to find related appendices for a given topic
 * @param {string} topic - The topic to find related appendices for
 * @returns {Array} Array of related appendix IDs
 */
export function findRelatedAppendices(topic) {
  // This would typically connect to an appendix index/database
  // For now, we'll implement a simple matching system
  const appendixMap = {
    'mathematics': ['mathematical-foundations'],
    'tools': ['tools-and-resources'],
    'datasets': ['datasets'],
    'examples': ['additional-examples'],
    // Add more mappings as needed
  };

  // Find all appendices related to the topic or matching keywords
  const related = [];
  Object.entries(appendixMap).forEach(([key, appendices]) => {
    if (topic.toLowerCase().includes(key) || key.includes(topic.toLowerCase())) {
      related.push(...appendices);
    }
  });

  return related;
}

/**
 * Function to generate links for related content
 * @param {Object} content - The content object to generate links for
 * @param {string} contentType - The type of content ('assessment', 'lab', 'glossary', 'appendix', 'module')
 * @returns {Object} Object containing links to related content
 */
export function generateRelatedContentLinks(content, contentType) {
  let relatedContent = {
    assessments: [],
    labs: [],
    glossary: [],
    appendices: []
  };

  switch (contentType) {
    case 'module':
      relatedContent.assessments = findRelatedAssessments(content.id);
      relatedContent.labs = findRelatedLabs(content.id);
      if (content.content) {
        relatedContent.glossary = findRelatedGlossaryTerms(content.content);
      }
      break;

    case 'assessment':
      relatedContent.glossary = findRelatedGlossaryTerms(content.questionText + ' ' + content.explanation);
      break;

    case 'lab':
      relatedContent.glossary = findRelatedGlossaryTerms(content.description + ' ' + content.steps.join(' '));
      break;

    case 'glossary':
      relatedContent.glossary = findRelatedGlossaryTerms(content.definition + ' ' + (content.examples ? content.examples.join(' ') : ''));
      break;

    case 'appendix':
      relatedContent.glossary = findRelatedGlossaryTerms(content.content);
      relatedContent.appendices = findRelatedAppendices(content.category);
      break;

    default:
      // For other content types, try to extract relevant terms
      if (content.content) {
        relatedContent.glossary = findRelatedGlossaryTerms(content.content);
      }
  }

  return relatedContent;
}

/**
 * Function to render related content links in a component
 * @param {Object} relatedContent - The related content object from generateRelatedContentLinks
 * @returns {JSX.Element} JSX for displaying related content links
 */
export function renderRelatedContentLinks(relatedContent) {
  return (
    <div className="related-content">
      {relatedContent.assessments.length > 0 && (
        <div className="related-assessments">
          <h4>Related Assessments:</h4>
          <ul>
            {relatedContent.assessments.map(id => (
              <li key={id}>
                <a href={`/docs/assessments/${id}`}>{id}</a>
              </li>
            ))}
          </ul>
        </div>
      )}
      
      {relatedContent.labs.length > 0 && (
        <div className="related-labs">
          <h4>Related Labs:</h4>
          <ul>
            {relatedContent.labs.map(id => (
              <li key={id}>
                <a href={`/docs/labs/${id}`}>{id}</a>
              </li>
            ))}
          </ul>
        </div>
      )}
      
      {relatedContent.glossary.length > 0 && (
        <div className="related-glossary">
          <h4>Related Glossary Terms:</h4>
          <ul>
            {relatedContent.glossary.map(id => (
              <li key={id}>
                <a href={`/docs/glossary/${id}`}>{id}</a>
              </li>
            ))}
          </ul>
        </div>
      )}
      
      {relatedContent.appendices.length > 0 && (
        <div className="related-appendices">
          <h4>Related Appendices:</h4>
          <ul>
            {relatedContent.appendices.map(id => (
              <li key={id}>
                <a href={`/docs/appendices/${id}`}>{id}</a>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}