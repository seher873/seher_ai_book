import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './Bookmark.module.css';

/**
 * Properties for the Bookmark component
 * @typedef {Object} BookmarkProps
 * @property {string} id - Unique identifier for the content being bookmarked
 * @property {string} title - Title of the content being bookmarked
 * @property {string} type - Type of content (glossaryTerm, appendixSection, module)
 */

/**
 * Bookmark component for saving frequently accessed content
 * @param {BookmarkProps} props - Component properties
 * @returns {JSX.Element} The bookmark component
 */
function Bookmark(props) {
  const { id, title, type } = props;
  const [bookmarked, setBookmarked] = useState(false);

  // Check if item is already bookmarked on component mount
  useEffect(() => {
    const bookmarks = JSON.parse(localStorage.getItem('textbookBookmarks') || '[]');
    const isBookmarked = bookmarks.some(bookmark => bookmark.id === id);
    setBookmarked(isBookmarked);
  }, [id]);

  const handleBookmarkToggle = () => {
    let bookmarks = JSON.parse(localStorage.getItem('textbookBookmarks') || '[]');
    
    if (bookmarked) {
      // Remove from bookmarks
      bookmarks = bookmarks.filter(bookmark => bookmark.id !== id);
    } else {
      // Add to bookmarks
      bookmarks.push({
        id,
        title,
        type,
        timestamp: new Date().toISOString()
      });
    }
    
    localStorage.setItem('textbookBookmarks', JSON.stringify(bookmarks));
    setBookmarked(!bookmarked);
  };

  return (
    <button
      onClick={handleBookmarkToggle}
      className={clsx(
        'button',
        bookmarked ? 'button--primary' : 'button--secondary',
        styles.bookmarkButton
      )}
      aria-label={bookmarked ? `Remove ${title} from bookmarks` : `Bookmark ${title}`}
    >
      {bookmarked ? '★ ' : '☆ '} {bookmarked ? 'Bookmarked' : 'Bookmark'}
    </button>
  );
}

export default Bookmark;