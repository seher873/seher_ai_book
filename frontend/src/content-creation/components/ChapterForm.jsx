import React, { useState } from 'react';
import SectionEditor from './SectionEditor';
import PreviewPane from './PreviewPane';

const ChapterForm = ({ chapter, onSave, onCancel }) => {
  const [formData, setFormData] = useState({
    title: chapter?.title || '',
    content: chapter?.content || '',
    learningObjectives: chapter?.learningObjectives || [''],
    sections: chapter?.sections || [{ title: '', content: '', type: 'concept' }],
    diagramPlaceholders: chapter?.diagramPlaceholders || [{ title: '', description: '' }],
    codeExamples: chapter?.codeExamples || [{ title: '', language: '', code: '', explanation: '' }]
  });

  const handleInputChange = (field, value) => {
    setFormData(prev => ({ ...prev, [field]: value }));
  };

  const handleSectionChange = (index, field, value) => {
    const newSections = [...formData.sections];
    newSections[index][field] = value;
    setFormData(prev => ({ ...prev, sections: newSections }));
  };

  const handleDiagramChange = (index, field, value) => {
    const newDiagrams = [...formData.diagramPlaceholders];
    newDiagrams[index][field] = value;
    setFormData(prev => ({ ...prev, diagramPlaceholders: newDiagrams }));
  };

  const handleCodeExampleChange = (index, field, value) => {
    const newCodeExamples = [...formData.codeExamples];
    newCodeExamples[index][field] = value;
    setFormData(prev => ({ ...prev, codeExamples: newCodeExamples }));
  };

  const addSection = () => {
    setFormData(prev => ({
      ...prev,
      sections: [...prev.sections, { title: '', content: '', type: 'concept' }]
    }));
  };

  const addDiagram = () => {
    setFormData(prev => ({
      ...prev,
      diagramPlaceholders: [...prev.diagramPlaceholders, { title: '', description: '' }]
    }));
  };

  const addCodeExample = () => {
    setFormData(prev => ({
      ...prev,
      codeExamples: [...prev.codeExamples, { title: '', language: '', code: '', explanation: '' }]
    }));
  };

  const removeSection = (index) => {
    if (formData.sections.length > 1) {
      const newSections = [...formData.sections];
      newSections.splice(index, 1);
      setFormData(prev => ({ ...prev, sections: newSections }));
    }
  };

  const removeDiagram = (index) => {
    if (formData.diagramPlaceholders.length > 1) {
      const newDiagrams = [...formData.diagramPlaceholders];
      newDiagrams.splice(index, 1);
      setFormData(prev => ({ ...prev, diagramPlaceholders: newDiagrams }));
    }
  };

  const removeCodeExample = (index) => {
    if (formData.codeExamples.length > 1) {
      const newCodeExamples = [...formData.codeExamples];
      newCodeExamples.splice(index, 1);
      setFormData(prev => ({ ...prev, codeExamples: newCodeExamples }));
    }
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSave(formData);
  };

  return (
    <div className="chapter-form-container">
      <h2>{chapter?.id ? 'Edit Chapter' : 'Create New Chapter'}</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="title">Chapter Title:</label>
          <input
            type="text"
            id="title"
            value={formData.title}
            onChange={(e) => handleInputChange('title', e.target.value)}
            required
          />
        </div>

        <div className="form-group">
          <label>Learning Objectives:</label>
          {formData.learningObjectives.map((objective, index) => (
            <div key={index} className="objective-input">
              <input
                type="text"
                value={objective}
                onChange={(e) => {
                  const newObjectives = [...formData.learningObjectives];
                  newObjectives[index] = e.target.value;
                  handleInputChange('learningObjectives', newObjectives);
                }}
                placeholder={`Learning objective ${index + 1}`}
              />
            </div>
          ))}
          <button type="button" onClick={() => handleInputChange('learningObjectives', [...formData.learningObjectives, ''])}>
            Add Objective
          </button>
        </div>

        <div className="form-group">
          <label>Sections:</label>
          {formData.sections.map((section, index) => (
            <div key={index} className="section-editor">
              <SectionEditor
                section={section}
                index={index}
                onChange={handleSectionChange}
                onRemove={removeSection}
              />
            </div>
          ))}
          <button type="button" onClick={addSection}>
            Add Section
          </button>
        </div>

        <div className="form-group">
          <label>Diagram Placeholders:</label>
          {formData.diagramPlaceholders.map((diagram, index) => (
            <div key={index} className="diagram-input">
              <input
                type="text"
                value={diagram.title}
                onChange={(e) => handleDiagramChange(index, 'title', e.target.value)}
                placeholder="Diagram title"
              />
              <textarea
                value={diagram.description}
                onChange={(e) => handleDiagramChange(index, 'description', e.target.value)}
                placeholder="Diagram description"
              />
              <button type="button" onClick={() => removeDiagram(index)}>Remove</button>
            </div>
          ))}
          <button type="button" onClick={addDiagram}>
            Add Diagram Placeholder
          </button>
        </div>

        <div className="form-group">
          <label>Code Examples:</label>
          {formData.codeExamples.map((codeExample, index) => (
            <div key={index} className="code-example-input">
              <input
                type="text"
                value={codeExample.title}
                onChange={(e) => handleCodeExampleChange(index, 'title', e.target.value)}
                placeholder="Code example title"
              />
              <select
                value={codeExample.language}
                onChange={(e) => handleCodeExampleChange(index, 'language', e.target.value)}
              >
                <option value="">Select language</option>
                <option value="javascript">JavaScript</option>
                <option value="python">Python</option>
                <option value="bash">Bash/Shell</option>
                <option value="markdown">Markdown</option>
                <option value="other">Other</option>
              </select>
              <textarea
                value={codeExample.code}
                onChange={(e) => handleCodeExampleChange(index, 'code', e.target.value)}
                placeholder="Code content"
                rows={4}
              />
              <textarea
                value={codeExample.explanation}
                onChange={(e) => handleCodeExampleChange(index, 'explanation', e.target.value)}
                placeholder="Explanation"
              />
              <button type="button" onClick={() => removeCodeExample(index)}>Remove</button>
            </div>
          ))}
          <button type="button" onClick={addCodeExample}>
            Add Code Example
          </button>
        </div>

        <div className="form-actions">
          <button type="submit">Save Chapter</button>
          <button type="button" onClick={onCancel}>Cancel</button>
        </div>
      </form>

      <div className="preview-pane">
        <PreviewPane content={formData} />
      </div>
    </div>
  );
};

export default ChapterForm;