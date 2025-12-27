import React, { useState, useMemo } from 'react';
import clsx from 'clsx';
import styles from './GlossarySearch.module.css';

/**
 * Properties for the GlossarySearch component
 * @typedef {Object} GlossarySearchProps
 * @property {Array} terms - Array of glossary terms to search through
 */

/**
 * GlossarySearch component for searching glossary terms
 * @param {GlossarySearchProps} props - Component properties
 * @returns {JSX.Element} The search component
 */
function GlossarySearch(props) {
  const { terms } = props;
  const [query, setQuery] = useState('');

  // Filter terms based on search query
  const filteredTerms = useMemo(() => {
    if (!query.trim()) {
      return [];
    }

    const results = terms.filter(term =>
      term.term.toLowerCase().includes(query.toLowerCase()) ||
      (term.definition && term.definition.toLowerCase().includes(query.toLowerCase())) ||
      (term.synonyms && term.synonyms.some(synonym => 
        synonym.toLowerCase().includes(query.toLowerCase())
      ))
    );

    return results.slice(0, 10); // Limit to 10 results
  }, [query, terms]);

  const handleClear = () => {
    setQuery('');
  };

  return (
    <div className={clsx('container', styles.searchContainer)}>
      <div className="row">
        <div className="col col--12">
          <div className={clsx('card', styles.searchCard)}>
            <div className="card__body">
              <div className={styles.searchHeader}>
                <h3>Glossary Search</h3>
              </div>
              
              <div className={styles.searchInputWrapper}>
                <input
                  type="text"
                  placeholder="Search glossary terms..."
                  value={query}
                  onChange={(e) => setQuery(e.target.value)}
                  className={clsx('form-control', styles.searchInput)}
                  aria-label="Search glossary terms"
                  role="searchbox"
                  aria-controls="search-results"
                  aria-expanded={query.length > 0}
                />
                {query && (
                  <button 
                    className={clsx('clean-btn', styles.clearButton)}
                    onClick={handleClear}
                    aria-label="Clear search"
                  >
                    ✕
                  </button>
                )}
              </div>
              
              {query && filteredTerms.length > 0 && (
                <ul id="search-results" className={clsx('list-unstyled', styles.searchResults)}>
                  {filteredTerms.map((term, index) => (
                    <li key={index} className={styles.searchResultItem}>
                      <a
                        href={`#${term.id}`}
                        className={styles.searchResultLink}
                        onClick={() => setQuery('')}
                      >
                        <span className={styles.resultTerm}>{term.term}</span>
                        <p className={styles.resultPreview}>
                          {term.definition.substring(0, 100)}{term.definition.length > 100 ? '...' : ''}
                        </p>
                      </a>
                    </li>
                  ))}
                </ul>
              )}
              
              {query && filteredTerms.length === 0 && (
                <div className={clsx('alert alert--warning', styles.noResults)}>
                  No terms found matching "{query}"
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default GlossarySearch;