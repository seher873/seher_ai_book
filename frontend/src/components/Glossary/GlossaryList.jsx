import React from 'react';
import clsx from 'clsx';
import styles from './GlossaryList.module.css';
import GlossaryTerm from './GlossaryTerm';

/**
 * Properties for the GlossaryList component
 * @typedef {Object} GlossaryListProps
 * @property {Array} terms - Array of glossary terms to display
 * @property {string} categoryFilter - Optional category to filter terms by
 */

/**
 * GlossaryList component for displaying multiple glossary terms
 * @param {GlossaryListProps} props - Component properties
 * @returns {JSX.Element} The glossary list component
 */
function GlossaryList(props) {
  const { terms, categoryFilter } = props;

  // Filter terms if a category is specified
  const filteredTerms = categoryFilter
    ? terms.filter(term => term.category === categoryFilter)
    : terms;

  return (
    <div className={styles.glossaryList}>
      {filteredTerms.length === 0 ? (
        <div className="alert alert--info">
          No glossary terms found{categoryFilter ? ` in the ${categoryFilter} category` : ''}.
        </div>
      ) : (
        filteredTerms.map((term, index) => (
          <GlossaryTerm
            key={index}
            id={term.id}
            term={term.term}
            definition={term.definition}
            category={term.category}
            relatedTerms={term.relatedTerms}
            examples={term.examples}
            synonyms={term.synonyms}
            seeAlso={term.seeAlso}
          />
        ))
      )}
    </div>
  );
}

export default GlossaryList;