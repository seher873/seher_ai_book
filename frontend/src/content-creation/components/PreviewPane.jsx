import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';

const PreviewPane = ({ content }) => {
  const renderLearningObjectives = () => {
    if (!content.learningObjectives || content.learningObjectives.length === 0) return null;
    
    return (
      <div className="objectives-preview">
        <h3>Learning Objectives</h3>
        <ul>
          {content.learningObjectives
            .filter(obj => obj.trim() !== '') // Filter out empty objectives
            .map((objective, index) => (
              <li key={index}>{objective}</li>
            ))}
        </ul>
      </div>
    );
  };

  const renderSections = () => {
    if (!content.sections || content.sections.length === 0) return null;

    return (
      <div className="sections-preview">
        <h3>Sections</h3>
        {content.sections.map((section, index) => (
          <div key={index} className="section-preview">
            <h4>{section.title || `Section ${index + 1}`}</h4>
            <div className="section-content">
              <ReactMarkdown remarkPlugins={[remarkGfm]}>{section.content}</ReactMarkdown>
            </div>
            <div className="section-type">Type: {section.type}</div>
          </div>
        ))}
      </div>
    );
  };

  const renderDiagrams = () => {
    if (!content.diagramPlaceholders || content.diagramPlaceholders.length === 0) return null;

    return (
      <div className="diagrams-preview">
        <h3>Diagram Placeholders</h3>
        {content.diagramPlaceholders.map((diagram, index) => (
          <div key={index} className="diagram-preview">
            <h5>{diagram.title}</h5>
            <p><em>Diagram Description:</em> {diagram.description}</p>
            <div className="diagram-placeholder">
              [Diagram Placeholder: {diagram.title}]
            </div>
          </div>
        ))}
      </div>
    );
  };

  const renderCodeExamples = () => {
    if (!content.codeExamples || content.codeExamples.length === 0) return null;

    return (
      <div className="code-examples-preview">
        <h3>Code Examples</h3>
        {content.codeExamples.map((example, index) => (
          <div key={index} className="code-example-preview">
            <h5>{example.title}</h5>
            <p><em>Language:</em> {example.language}</p>
            <div className="code-block">
              <pre><code>{example.code}</code></pre>
            </div>
            <p><em>Explanation:</em> {example.explanation}</p>
          </div>
        ))}
      </div>
    );
  };

  return (
    <div className="preview-pane">
      <h2>Preview: {content.title || 'Untitled Chapter'}</h2>
      
      {renderLearningObjectives()}
      
      <div className="main-content-preview">
        <ReactMarkdown remarkPlugins={[remarkGfm]}>{content.content}</ReactMarkdown>
      </div>
      
      {renderSections()}
      {renderDiagrams()}
      {renderCodeExamples()}
    </div>
  );
};

export default PreviewPane;