import React from 'react';
import clsx from 'clsx';
import styles from './LabEmbed.module.css';

/**
 * Properties for the LabEmbed component
 * @typedef {Object} LabEmbedProps
 * @property {string} notebookUrl - URL to the cloud-based notebook environment
 * @property {string} title - Title of the lab exercise
 * @property {string} description - Brief description of what the exercise covers
 */

/**
 * LabEmbed component for embedding cloud notebook environments
 * @param {LabEmbedProps} props - Component properties
 * @returns {JSX.Element} The lab embed component
 */
function LabEmbed(props) {
  const { notebookUrl, title, description } = props;

  return (
    <div className={clsx('card', styles.labEmbedCard)}>
      <div className="card__header">
        <h3>Lab Environment: {title}</h3>
      </div>
      <div className="card__body">
        <p>{description}</p>
        <p>
          This lab uses a cloud-based Jupyter notebook environment. No local installation required.
        </p>
        <div className={styles.notebookContainer}>
          <a 
            href={notebookUrl} 
            target="_blank" 
            rel="noopener noreferrer"
            className={clsx('button button--primary button--lg', styles.notebookButton)}
          >
            Open in Notebook
          </a>
        </div>
        <div className={styles.notebookInfo}>
          <p><strong>Requirements:</strong> Internet connection and a modern web browser</p>
          <p><strong>Estimated time:</strong> As specified in the lab instructions</p>
        </div>
      </div>
    </div>
  );
}

export default LabEmbed;