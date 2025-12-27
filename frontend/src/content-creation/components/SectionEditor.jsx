import React from 'react';

const SectionEditor = ({ section, index, onChange, onRemove }) => {
  return (
    <div className="section-editor-wrapper">
      <div className="section-controls">
        <h4>Section {index + 1}</h4>
        <button type="button" onClick={() => onRemove(index)}>Remove Section</button>
      </div>
      
      <div className="section-fields">
        <div className="form-group">
          <label htmlFor={`section-title-${index}`}>Title:</label>
          <input
            id={`section-title-${index}`}
            type="text"
            value={section.title}
            onChange={(e) => onChange(index, 'title', e.target.value)}
            placeholder="Section title"
          />
        </div>
        
        <div className="form-group">
          <label htmlFor={`section-content-${index}`}>Content:</label>
          <textarea
            id={`section-content-${index}`}
            value={section.content}
            onChange={(e) => onChange(index, 'content', e.target.value)}
            placeholder="Section content in Markdown format"
            rows={6}
          />
        </div>
        
        <div className="form-group">
          <label htmlFor={`section-type-${index}`}>Type:</label>
          <select
            id={`section-type-${index}`}
            value={section.type}
            onChange={(e) => onChange(index, 'type', e.target.value)}
          >
            <option value="concept">Concept</option>
            <option value="tutorial">Tutorial</option>
            <option value="example">Example</option>
            <option value="summary">Summary</option>
            <option value="workflow">Workflow</option>
          </select>
        </div>
      </div>
    </div>
  );
};

export default SectionEditor;