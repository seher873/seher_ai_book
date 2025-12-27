import React from 'react';
import clsx from 'clsx';
import styles from './AppendixSection.module.css';

/**
 * Properties for the AppendixSection component
 * @typedef {Object} AppendixSectionProps
 * @property {string} id - Unique identifier for the appendix section
 * @property {string} title - Title of the appendix section
 * @property {string} content - Main content of the appendix (Markdown format)
 * @property {string} category - Topic category (e.g., "tools", "mathematical-foundations", "datasets", "chatbot-rag")
 * @property {string} skillLevel - Target skill level (beginner, intermediate, advanced)
 * @property {Array<string>} relatedModules - Array of module IDs this appendix connects to
 * @property {Array<string>} tags - Array of tags for easy search
 * @property {Array<string>} references - Array of references cited in the appendix
 */

/**
 * AppendixSection component for displaying appendix content
 * @param {AppendixSectionProps} props - Component properties
 * @returns {JSX.Element} The appendix section component
 */
function AppendixSection(props) {
  const {
    title,
    content = '',
    category,
    skillLevel,
    relatedModules = [],
    tags = [],
    references = []
  } = props;

  return (
    <div className={clsx('card', styles.appendixSectionCard)}>
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
          dangerouslySetInnerHTML={{ __html: content }}
        />

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

        {tags.length > 0 && (
          <section className={styles.section}>
            <h3>Tags</h3>
            <div className={styles.tagsContainer}>
              {tags.map((tag, index) => (
                <span key={index} className="badge badge--info margin-right--sm">
                  {tag}
                </span>
              ))}
            </div>
          </section>
        )}
      </div>
    </div>
  );
}

export default AppendixSection;