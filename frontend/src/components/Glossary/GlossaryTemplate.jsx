import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './GlossaryTemplate.module.css';

/**
 * Properties for the GlossaryTemplate component
 * @typedef {Object} GlossaryProps
 * @property {string} id - Unique identifier for the term
 * @property {string} term - The actual term being defined
 * @property {string} definition - The definition of the term
 * @property {string} category - Topic category (e.g., "algorithms", "ethics", "tools")
 * @property {string[]} relatedTerms - Array of related term IDs
 * @property {string[]} examples - Array of examples that illustrate the term
 * @property {string[]} synonyms - Array of alternative terms with similar meanings
 * @property {string[]} seeAlso - Array of references to related concepts in the textbook
 */

/**
 * GlossaryTemplate component for displaying glossary terms
 * @param {GlossaryProps} props - Component properties
 * @returns {JSX.Element} The glossary component
 */
function GlossaryTemplate(props) {
  const {
    term,
    definition,
    category,
    relatedTerms = [],
    examples = [],
    synonyms = [],
    seeAlso = []
  } = props;

  const [isExpanded, setIsExpanded] = useState(false);

  const toggleExpanded = () => {
    setIsExpanded(!isExpanded);
  };

  return (
    <div className={clsx('card', styles.glossaryCard)}>
      <div className="card__header">
        <h3>{term}</h3>
        {category && (
          <span className="badge badge--primary">
            {category}
          </span>
        )}
      </div>
      
      <div className="card__body">
        <p><strong>Definition:</strong> {definition}</p>
        
        {examples.length > 0 && (
          <details className={styles.details}>
            <summary>Examples</summary>
            <ul>
              {examples.map((example, index) => (
                <li key={index}>{example}</li>
              ))}
            </ul>
          </details>
        )}
        
        {synonyms.length > 0 && (
          <details className={styles.details}>
            <summary>Synonyms</summary>
            <ul>
              {synonyms.map((synonym, index) => (
                <li key={index}>{synonym}</li>
              ))}
            </ul>
          </details>
        )}
        
        {isExpanded && (
          <>
            {relatedTerms.length > 0 && (
              <details className={styles.details} open>
                <summary>Related Terms</summary>
                <ul>
                  {relatedTerms.map((relatedTerm, index) => (
                    <li key={index}>{relatedTerm}</li>
                  ))}
                </ul>
              </details>
            )}
            
            {seeAlso.length > 0 && (
              <details className={styles.details} open>
                <summary>See Also</summary>
                <ul>
                  {seeAlso.map((reference, index) => (
                    <li key={index}>{reference}</li>
                  ))}
                </ul>
              </details>
            )}
          </>
        )}
        
        {relatedTerms.length > 0 || seeAlso.length > 0 ? (
          <button 
            className={clsx('button button--secondary button--sm', styles.toggleButton)}
            onClick={toggleExpanded}
          >
            {isExpanded ? 'Show Less' : 'Show More'}
          </button>
        ) : null}
      </div>
    </div>
  );
}

export default GlossaryTemplate;