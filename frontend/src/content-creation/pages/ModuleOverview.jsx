import React, { useState, useEffect } from 'react';
import { Link } from 'react-router-dom';

const ModuleOverview = () => {
  const [moduleData, setModuleData] = useState({
    id: 'module-4',
    title: 'Module 4: LLMs + Robotics - Voice-to-Action Systems',
    description: 'This module covers the integration of Large Language Models with robotics for voice-to-action systems, focusing on converting natural language commands into ROS 2-based robotic actions including perception, navigation, and manipulation.',
    authors: ['AI Educator'],
    status: 'in-progress',
    createdAt: '2025-12-14',
    updatedAt: '2025-12-14',
    learningObjectives: [
      'Understand Vision-Language-Action (VLA) models and their applications',
      'Convert voice commands to text for robotic processing',
      'Decompose natural language commands into executable robotic actions',
      'Understand multimodal perception systems',
      'Implement ROS 2 planning and execution for voice commands',
      'Build a complete capstone autonomous humanoid system'
    ],
    chapters: [
      { id: 'index', title: 'Introduction', status: 'completed', wordCount: 1200, lastModified: '2025-12-14' },
      { id: '01-intro-vla', title: 'Chapter 1: Intro VLA', status: 'completed', wordCount: 1800, lastModified: '2025-12-14' },
      { id: '02-speech-to-text', title: 'Chapter 2: Speech-to-Text', status: 'completed', wordCount: 2200, lastModified: '2025-12-14' },
      { id: '03-task-decomposition', title: 'Chapter 3: Task Decomposition', status: 'in-progress', wordCount: 1500, lastModified: '2025-12-14' },
      { id: '04-multimodal-perception', title: 'Chapter 4: Multimodal Perception', status: 'not-started', wordCount: 0, lastModified: null },
      { id: '05-ros2-planning-execution', title: 'Chapter 5: ROS 2 Planning & Execution', status: 'not-started', wordCount: 0, lastModified: null },
      { id: '06-capstone-humanoid', title: 'Chapter 6: Capstone Autonomous Humanoid', status: 'not-started', wordCount: 0, lastModified: null }
    ]
  });

  const [stats, setStats] = useState({
    totalChapters: 7,
    completedChapters: 3,
    inProgressChapters: 1,
    notStartedChapters: 3,
    totalWordCount: 6700,
    avgWordsPerChapter: 957
  });

  const getStatusColor = (status) => {
    switch (status) {
      case 'completed': return '#4CAF50';
      case 'in-progress': return '#FFC107';
      case 'not-started': return '#9E9E9E';
      default: return '#9E9E9E';
    }
  };

  const getStatsForStatus = (status) => {
    return moduleData.chapters.filter(c => c.status === status).length;
  };

  return (
    <div className="module-overview-container">
      <div className="module-header">
        <h1>{moduleData.title}</h1>
        <p>{moduleData.description}</p>
        <div className="module-meta">
          <span>Created: {moduleData.createdAt}</span>
          <span> | Updated: {moduleData.updatedAt}</span>
          <span> | Status: <span style={{ color: getStatusColor(moduleData.status) }}>{moduleData.status}</span></span>
        </div>
      </div>

      <div className="module-stats">
        <div className="stat-card">
          <h3>Chapters</h3>
          <p>Total: {stats.totalChapters}</p>
          <p>Completed: {stats.completedChapters}</p>
          <p>In Progress: {stats.inProgressChapters}</p>
          <p>Not Started: {stats.notStartedChapters}</p>
        </div>
        
        <div className="stat-card">
          <h3>Content</h3>
          <p>Total Words: {stats.totalWordCount.toLocaleString()}</p>
          <p>Avg/Chapter: {stats.avgWordsPerChapter.toLocaleString()}</p>
        </div>
        
        <div className="stat-card">
          <h3>Progress</h3>
          <div className="progress-circle">
            <div className="progress-text">{Math.round((stats.completedChapters / stats.totalChapters) * 100)}%</div>
          </div>
        </div>
      </div>

      <div className="learning-objectives">
        <h2>Learning Objectives</h2>
        <ul>
          {moduleData.learningObjectives.map((objective, index) => (
            <li key={index}>{objective}</li>
          ))}
        </ul>
      </div>

      <div className="chapters-overview">
        <h2>Chapters</h2>
        <table className="chapters-table">
          <thead>
            <tr>
              <th>Chapter</th>
              <th>Title</th>
              <th>Status</th>
              <th>Word Count</th>
              <th>Last Modified</th>
              <th>Actions</th>
            </tr>
          </thead>
          <tbody>
            {moduleData.chapters.map((chapter, index) => (
              <tr key={chapter.id}>
                <td>{index === 0 ? 'Introduction' : `Chapter ${index}`}</td>
                <td>
                  <Link to={`/chapter-editor/${chapter.id}`}>{chapter.title}</Link>
                </td>
                <td>
                  <span 
                    style={{ 
                      color: getStatusColor(chapter.status),
                      fontWeight: 'bold' 
                    }}
                  >
                    {chapter.status.replace('-', ' ')}
                  </span>
                </td>
                <td>{chapter.wordCount}</td>
                <td>{chapter.lastModified || '-'}</td>
                <td>
                  <Link to={`/chapter-editor/${chapter.id}`} className="btn btn-small btn-primary">
                    Edit
                  </Link>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <div className="module-actions">
        <Link to="/dashboard" className="btn btn-secondary">Back to Dashboard</Link>
        <Link to="/chapter-editor/new" className="btn btn-primary">Add New Chapter</Link>
        <button className="btn btn-tertiary">Export Module</button>
        <button className="btn btn-tertiary">Validate Module</button>
      </div>
    </div>
  );
};

export default ModuleOverview;