import React, { useState, useEffect } from 'react';
import { useParams, useNavigate } from 'react-router-dom';
import ChapterForm from '../components/ChapterForm';

const ChapterEditor = () => {
  const { chapterId } = useParams();
  const navigate = useNavigate();
  const [chapter, setChapter] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    // Simulate loading chapter data
    const fetchChapter = async () => {
      try {
        // In a real implementation, this would fetch from an API
        // For now, we'll simulate with dummy data or empty structure
        await new Promise(resolve => setTimeout(resolve, 300));
        
        if (chapterId && chapterId !== 'new') {
          // Load existing chapter (dummy data for simulation)
          setChapter({
            id: chapterId,
            title: chapterId === 'index' ? 'Module 4 - LLMs + Robotics: Voice-to-Action Systems' : 
                  chapterId === '01-intro-vla' ? 'Chapter 1 - Intro VLA' :
                  chapterId === '02-speech-to-text' ? 'Chapter 2 - Speech-to-Text' :
                  chapterId === '03-task-decomposition' ? 'Chapter 3 - Task Decomposition' :
                  chapterId === '04-multimodal-perception' ? 'Chapter 4 - Multimodal Perception' :
                  chapterId === '05-ros2-planning-execution' ? 'Chapter 5 - ROS 2 Planning & Execution' :
                  'Chapter 6 - Capstone Autonomous Humanoid',
            content: '# Chapter Content\n\nThis is the main content of the chapter.',
            learningObjectives: ['Understand key concepts', 'Apply knowledge to practical examples'],
            sections: [
              { title: 'Introduction', content: 'This section introduces the topic...', type: 'concept' },
              { title: 'Detailed Explanation', content: 'This section provides detailed information...', type: 'tutorial' }
            ],
            diagramPlaceholders: [
              { title: 'System Architecture', description: 'Diagram showing the system architecture' }
            ],
            codeExamples: [
              { title: 'Example Code', language: 'python', code: 'print("Hello, World!")', explanation: 'A simple example' }
            ]
          });
        } else {
          // New chapter
          setChapter({
            id: null,
            title: '',
            content: '',
            learningObjectives: [''],
            sections: [{ title: '', content: '', type: 'concept' }],
            diagramPlaceholders: [{ title: '', description: '' }],
            codeExamples: [{ title: '', language: '', code: '', explanation: '' }]
          });
        }
        
        setLoading(false);
      } catch (err) {
        setError('Failed to load chapter');
        setLoading(false);
      }
    };

    fetchChapter();
  }, [chapterId]);

  const handleSave = async (chapterData) => {
    try {
      // In a real implementation, this would send to an API
      // For simulation, we'll just navigate back to dashboard
      console.log('Saving chapter:', chapterData);
      
      // Show success message
      alert('Chapter saved successfully!');
      
      // Navigate back to dashboard
      navigate('/dashboard');
    } catch (err) {
      setError('Failed to save chapter');
      console.error(err);
    }
  };

  const handleCancel = () => {
    navigate('/dashboard');
  };

  if (loading) {
    return <div>Loading chapter editor...</div>;
  }

  if (error) {
    return <div>Error: {error}</div>;
  }

  return (
    <div className="chapter-editor-container">
      <div className="editor-header">
        <h1>{chapterId === 'new' ? 'Create New Chapter' : `Edit Chapter: ${chapter.title}`}</h1>
        <button className="btn btn-secondary" onClick={() => navigate('/dashboard')}>
          Back to Dashboard
        </button>
      </div>
      
      <ChapterForm 
        chapter={chapter} 
        onSave={handleSave} 
        onCancel={handleCancel} 
      />
    </div>
  );
};

export default ChapterEditor;