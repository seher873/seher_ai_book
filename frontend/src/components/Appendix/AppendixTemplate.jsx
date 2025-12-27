import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './AppendixTemplate.module.css';

/**
 * Properties for the AppendixTemplate component
 * @typedef {Object} AppendixProps
 * @property {string} id - Unique identifier for the appendix
 * @property {string} title - Title of the appendix
 * @property {string} content - Main content of the appendix (Markdown format)
 * @property {string} category - Topic category (e.g., "tools", "mathematical-foundations", "datasets")
 * @property {string} skillLevel - Target skill level (beginner, intermediate, advanced)
 * @property {string[]} relatedModules - Array of module IDs this appendix connects to
 * @property {string[]} tags - Array of tags for easy search
 * @property {string[]} references - Array of references cited in the appendix
 */

/**
 * AppendixTemplate component for displaying appendix content
 * @param {AppendixProps} props - Component properties
 * @returns {JSX.Element} The appendix component
 */
function AppendixTemplate(props) {
  const [showAll, setShowAll] = useState(false);

  const {
    title,
    content = '',
    category,
    skillLevel,
    relatedModules = [],
    tags = [],
    references = []
  } = props;

  // Simple function to truncate content if too long
  const displayContent = showAll || content.length <= 500 
    ? content 
    : content.substring(0, 500) + '...';

  return (
    <div className={clsx('card', styles.appendixCard)}>
      <div className="card__header">
        <h2>{title}</h2>
        <div className={styles.headerBadges}>
          {category && (
            <span className="badge badge--primary">
              {category}
            </span>
          )}
          {skillLevel && (
            <span className={clsx('badge', `badge--${skillLevel}`)}>
              {skillLevel}
            </span>
          )}
        </div>
      </div>
      
      <div className="card__body">
        <div 
          className={styles.content}
          dangerouslySetInnerHTML={{ __html: displayContent }} 
        />
        
        {content.length > 500 && (
          <button 
            className={clsx('button button--secondary', styles.toggleButton)}
            onClick={() => setShowAll(!showAll)}
          >
            {showAll ? 'Show Less' : 'Show More'}
          </button>
        )}
        
        {relatedModules.length > 0 && (
          <section className={styles.section}>
            <h3>Related Modules</h3>
            <ul>
              {relatedModules.map((module, index) => (
                <li key={index}>
                  <a href={`/docs/${module}`}>{module}</a>
                </li>
              ))}
            </ul>
          </section>
        )}
        
        {references.length > 0 && (
          <section className={styles.section}>
            <h3>References</h3>
            <ul>
              {references.map((ref, index) => (
                <li key={index} className={styles.referenceItem}>{ref}</li>
              ))}
            </ul>
          </section>
        )}
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

export default AppendixTemplate;