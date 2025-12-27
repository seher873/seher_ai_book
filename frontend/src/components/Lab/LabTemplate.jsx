import React from 'react';
import clsx from 'clsx';
import styles from './LabTemplate.module.css';

/**
 * Properties for the LabTemplate component
 * @typedef {Object} LabProps
 * @property {string} id - Unique identifier for the lab exercise
 * @property {string} title - Title of the exercise
 * @property {string} description - Brief description of what the exercise covers
 * @property {string[]} prerequisites - Array of module IDs or skills required
 * @property {number} estimatedDuration - Time estimate in minutes
 * @property {string} notebookUrl - URL to the cloud-based notebook environment
 * @property {string[]} objectives - Array of learning objectives
 * @property {string[]} steps - Ordered array of instructions for the exercise
 * @property {string[]} datasets - Array of dataset names or URLs used in the exercise
 * @property {string} hardwareRequirements - Reference to hardware profile
 * @property {string} difficultyLevel - Level of difficulty
 * @property {string[]} tags - Array of tags for categorizing the exercise
 */

/**
 * LabTemplate component for displaying lab exercises
 * @param {LabProps} props - Component properties
 * @returns {JSX.Element} The lab component
 */
function LabTemplate(props) {
  const {
    title,
    description,
    prerequisites = [],
    estimatedDuration,
    notebookUrl,
    objectives = [],
    steps = [],
    datasets = [],
    hardwareRequirements,
    difficultyLevel,
    tags = []
  } = props;

  return (
    <div className={clsx('card', styles.labCard)}>
      <div className="card__header">
        <h2>{title}</h2>
        <div className={styles.headerInfo}>
          <span className={clsx('badge', `badge--${difficultyLevel}`)}>
            {difficultyLevel}
          </span>
          <span className="badge badge--secondary">
            ~{estimatedDuration} min
          </span>
        </div>
      </div>
      
      <div className="card__body">
        <p>{description}</p>
        
        {objectives.length > 0 && (
          <section className={styles.section}>
            <h3>Learning Objectives</h3>
            <ul>
              {objectives.map((objective, index) => (
                <li key={index}>{objective}</li>
              ))}
            </ul>
          </section>
        )}
        
        {prerequisites.length > 0 && (
          <section className={styles.section}>
            <h3>Prerequisites</h3>
            <ul>
              {prerequisites.map((prereq, index) => (
                <li key={index}>{prereq}</li>
              ))}
            </ul>
          </section>
        )}
        
        <section className={styles.section}>
          <h3>Getting Started</h3>
          <p>
            This lab uses a cloud-based Jupyter notebook environment.{' '}
            <a 
              href={notebookUrl} 
              target="_blank" 
              rel="noopener noreferrer"
              className={clsx('button button--primary', styles.notebookLink)}
            >
              Open in Notebook
            </a>
          </p>
        </section>
        
        {datasets.length > 0 && (
          <section className={styles.section}>
            <h3>Datasets Used</h3>
            <ul>
              {datasets.map((dataset, index) => (
                <li key={index}>{dataset}</li>
              ))}
            </ul>
          </section>
        )}
        
        {hardwareRequirements && (
          <section className={styles.section}>
            <h3>Hardware Requirements</h3>
            <p>{hardwareRequirements}</p>
          </section>
        )}
        
        <section className={styles.section}>
          <h3>Lab Steps</h3>
          <ol className={styles.stepsList}>
            {steps.map((step, index) => (
              <li key={index}>{step}</li>
            ))}
          </ol>
        </section>
      </div>
      
      {tags.length > 0 && (
        <div className="card__footer">
          {tags.map((tag, index) => (
            <span key={index} className="badge badge--info margin-right--sm">
              {tag}
            </span>
          ))}
        </div>
      )}
    </div>
  );
}

export default LabTemplate;