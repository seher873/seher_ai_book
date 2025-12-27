import React from 'react';
import clsx from 'clsx';
import styles from './NavigationLink.module.css';

/**
 * Properties for the NavigationLink component
 * @typedef {Object} NavigationLinkProps
 * @property {string} id - Unique identifier for the navigation link
 * @property {string} label - Text to display for the link
 * @property {string} url - URL path to the target location
 * @property {string} sourceSection - Section where the link is placed (mainContent, glossary, appendices)
 * @property {string} targetSection - Section the link navigates to (mainContent, glossary, appendices)
 * @property {number} displayPriority - Priority for display order (lower numbers first)
 */

/**
 * NavigationLink component for linking between different sections of the textbook
 * @param {NavigationLinkProps} props - Component properties
 * @returns {JSX.Element} The navigation link component
 */
function NavigationLink(props) {
  const { label, url, displayPriority, sourceSection, targetSection } = props;

  // Determine appropriate styling based on source/target sections
  const getLinkStyle = () => {
    if (sourceSection === 'mainContent' && targetSection === 'glossary') {
      return styles.mainToGlossary;
    } else if (sourceSection === 'mainContent' && targetSection === 'appendices') {
      return styles.mainToAppendix;
    } else if (sourceSection === 'glossary' && targetSection === 'mainContent') {
      return styles.glossaryToMain;
    } else if (sourceSection === 'appendices' && targetSection === 'mainContent') {
      return styles.appendixToMain;
    }
    return '';
  };

  return (
    <a
      href={url}
      className={clsx(
        'button button--secondary button--md',
        styles.navLink,
        getLinkStyle()
      )}
      style={{ order: displayPriority }}
    >
      {label}
    </a>
  );
}

export default NavigationLink;