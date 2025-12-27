import React from 'react';
import clsx from 'clsx';
import styles from './AppendixNavigation.module.css';

/**
 * Properties for the AppendixNavigation component
 * @typedef {Object} AppendixNavigationProps
 * @property {Array} sections - Array of appendix sections with id and title
 * @property {string} currentSectionId - ID of the currently displayed section
 */

/**
 * AppendixNavigation component for navigating between appendix sections
 * @param {AppendixNavigationProps} props - Component properties
 * @returns {JSX.Element} The appendix navigation component
 */
function AppendixNavigation(props) {
  const { sections = [], currentSectionId } = props;

  return (
    <div className={clsx('card', styles.navigationCard)}>
      <div className="card__header">
        <h3>Appendix Sections</h3>
      </div>
      <div className="card__body">
        <nav className={styles.navList}>
          <ul>
            {sections.map((section, index) => (
              <li key={index} className={styles.navItem}>
                <a
                  href={`#${section.id}`}
                  className={clsx(styles.navLink, {
                    [styles.current]: currentSectionId === section.id
                  })}
                >
                  {section.title}
                </a>
              </li>
            ))}
          </ul>
        </nav>
      </div>
    </div>
  );
}

export default AppendixNavigation;