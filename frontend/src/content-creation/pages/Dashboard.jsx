import React, { useState, useEffect } from 'react';
import { Link } from 'react-router-dom';

const Dashboard = () => {
  const [modules, setModules] = useState([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Simulate loading modules data
    const fetchModules = async () => {
      // In a real implementation, this would fetch from an API
      // For now, we'll simulate with dummy data
      await new Promise(resolve => setTimeout(resolve, 500));
      
      setModules([
        {
          id: 'module-4',
          title: 'Module 4: LLMs + Robotics - Voice-to-Action Systems',
          chapters: [
            { id: 'index', title: 'Introduction', status: 'completed' },
            { id: '01-intro-vla', title: 'Chapter 1: Intro VLA', status: 'completed' },
            { id: '02-speech-to-text', title: 'Chapter 2: Speech-to-Text', status: 'completed' },
            { id: '03-task-decomposition', title: 'Chapter 3: Task Decomposition', status: 'in-progress' },
            { id: '04-multimodal-perception', title: 'Chapter 4: Multimodal Perception', status: 'not-started' },
            { id: '05-ros2-planning-execution', title: 'Chapter 5: ROS 2 Planning & Execution', status: 'not-started' },
            { id: '06-capstone-humanoid', title: 'Chapter 6: Capstone Autonomous Humanoid', status: 'not-started' }
          ],
          completedChapters: 3,
          totalChapters: 7,
          lastUpdated: '2025-12-14'
        }
      ]);
      
      setLoading(false);
    };

    fetchModules();
  }, []);

  const getStatusColor = (status) => {
    switch (status) {
      case 'completed': return 'green';
      case 'in-progress': return 'orange';
      case 'not-started': return 'gray';
      default: return 'gray';
    }
  };

  if (loading) {
    return <div>Loading dashboard...</div>;
  }

  return (
    <div className="dashboard-container">
      <h1>Module 4 Content Creation Dashboard</h1>
      
      <div className="module-overview">
        <h2>Module Overview</h2>
        {modules.map(module => (
          <div key={module.id} className="module-summary">
            <h3>{module.title}</h3>
            <div className="progress-summary">
              <p>
                Progress: {module.completedChapters}/{module.totalChapters} chapters completed
              </p>
              <div className="progress-bar">
                <div 
                  className="progress-fill" 
                  style={{ width: `${(module.completedChapters / module.totalChapters) * 100}%` }}
                ></div>
              </div>
            </div>
            <p>Last updated: {module.lastUpdated}</p>
          </div>
        ))}
      </div>

      <div className="chapters-list">
        <h2>Chapters</h2>
        {modules[0]?.chapters.map((chapter, index) => (
          <div key={chapter.id} className="chapter-card">
            <div className="chapter-info">
              <span className="chapter-number">Chapter {index > 0 ? index : 'Introduction'}</span>
              <h3>
                <Link to={`/chapter-editor/${chapter.id}`}>{chapter.title}</Link>
              </h3>
            </div>
            <div className="chapter-status" style={{ color: getStatusColor(chapter.status) }}>
              {chapter.status.replace('-', ' ')}
            </div>
          </div>
        ))}
      </div>

      <div className="actions">
        <Link to="/chapter-editor/new" className="btn btn-primary">
          Create New Chapter
        </Link>
        <button className="btn btn-secondary">Export Module Content</button>
        <button className="btn btn-tertiary">Import Content</button>
      </div>
    </div>
  );
};

export default Dashboard;