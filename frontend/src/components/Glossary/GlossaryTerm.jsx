import React from 'react';
import clsx from 'clsx';
import styles from './GlossaryTerm.module.css';

/**
 * Properties for the GlossaryTerm component
 * @typedef {Object} GlossaryTermProps
 * @property {string} id - Unique identifier for the term
 * @property {string} term - The actual term being defined
 * @property {string} definition - The definition of the term
 * @property {string} category - Topic category (e.g., "algorithms", "ethics", "tools")
 * @property {Array<string>} relatedTerms - Array of related term IDs
 * @property {Array<string>} examples - Array of examples that illustrate the term
 * @property {Array<string>} synonyms - Array of alternative terms with similar meanings
 * @property {Array<string>} seeAlso - Array of references to related concepts in the textbook
 */

/**
 * GlossaryTerm component for displaying individual glossary terms
 * @param {GlossaryTermProps} props - Component properties
 * @returns {JSX.Element} The glossary term component
 */
function GlossaryTerm(props) {
  const {
    id,
    term,
    definition,
    category,
    relatedTerms = [],
    examples = [],
    synonyms = [],
    seeAlso = []
  } = props;

  return (
    <div id={id} className={clsx('card', styles.glossaryTermCard)}>
      <div className="card__header">
        <h3 className={styles.termTitle}>{term}</h3>
        {category && (
          <span className="badge badge--secondary">
            {category}
          </span>
        )}
      </div>

      <div className="card__body">
        <div className={styles.definitionSection}>
          <h4>Definition</h4>
          <p>{definition}</p>
        </div>

        {examples.length > 0 && (
          <div className={styles.examplesSection}>
            <h4>Examples</h4>
            <ul>
              {examples.map((example, index) => (
                <li key={index}>{example}</li>
              ))}
            </ul>
          </div>
        )}

        {synonyms.length > 0 && (
          <div className={styles.synonymsSection}>
            <h4>Synonyms</h4>
            <ul>
              {synonyms.map((synonym, index) => (
                <li key={index}>{synonym}</li>
              ))}
            </ul>
          </div>
        )}

        {relatedTerms.length > 0 && (
          <div className={styles.relatedTermsSection}>
            <h4>Related Terms</h4>
            <ul>
              {relatedTerms.map((relatedTerm, index) => (
                <li key={index}>{relatedTerm}</li>
              ))}
            </ul>
          </div>
        )}

        {seeAlso.length > 0 && (
          <div className={styles.seeAlsoSection}>
            <h4>See Also</h4>
            <ul>
              {seeAlso.map((reference, index) => (
                <li key={index}>{reference}</li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </div>
  );
}

export default GlossaryTerm;