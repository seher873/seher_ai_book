/**
 * Utility functions for formatting and converting data structures
 * within the Module 4 content creation interface
 */

/**
 * Formats a date object or timestamp into a standard format
 * @param {Date|string|number} date - The date to format
 * @param {string} [format='YYYY-MM-DD'] - The format to use
 * @returns {string} The formatted date string
 */
export function formatDate(date, format = 'YYYY-MM-DD') {
  if (!date) {
    return '';
  }

  let dateObj;
  if (typeof date === 'string' || typeof date === 'number') {
    dateObj = new Date(date);
  } else if (date instanceof Date) {
    dateObj = date;
  } else {
    return '';
  }

  if (isNaN(dateObj.getTime())) {
    return '';
  }

  const year = dateObj.getFullYear();
  const month = String(dateObj.getMonth() + 1).padStart(2, '0');
  const day = String(dateObj.getDate()).padStart(2, '0');

  if (format === 'YYYY-MM-DD') {
    return `${year}-${month}-${day}`;
  } else if (format === 'MM/DD/YYYY') {
    return `${month}/${day}/${year}`;
  } else if (format === 'DD/MM/YYYY') {
    return `${day}/${month}/${year}`;
  } else if (format === 'MMM DD, YYYY') {
    const months = [
      'Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun',
      'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'
    ];
    const monthName = months[dateObj.getMonth()];
    return `${monthName} ${day}, ${year}`;
  }

  // Default to YYYY-MM-DD
  return `${year}-${month}-${day}`;
}

/**
 * Formats a string as a valid filename
 * @param {string} str - The string to format
 * @returns {string} The formatted filename
 */
export function formatAsFilename(str) {
  if (!str) return '';

  // Remove special characters and replace spaces with hyphens
  return str
    .toLowerCase()
    .replace(/[^\w\s-]/g, '') // Remove special characters
    .replace(/[\s_-]+/g, '-') // Replace multiple spaces or hyphens with a single hyphen
    .replace(/^-+|-+$/g, '') // Remove leading/trailing hyphens
    .substring(0, 100); // Limit length to 100 characters
}

/**
 * Converts a camelCase or PascalCase string to kebab-case
 * @param {string} str - The string to convert
 * @returns {string} The converted string
 */
export function camelToKebab(str) {
  if (!str) return '';
  
  // Insert a dash before uppercase letters that follow lowercase letters
  return str
    .replace(/([a-z])([A-Z])/g, '$1-$2')
    .replace(/[\s_]+/g, '-')
    .toLowerCase();
}

/**
 * Converts a string to title case
 * @param {string} str - The string to convert
 * @returns {string} The title case string
 */
export function toTitleCase(str) {
  if (!str) return '';
  
  return str.toLowerCase().split(' ').map(word => {
    if (word.length === 0) return word;
    return word.charAt(0).toUpperCase() + word.slice(1);
  }).join(' ');
}

/**
 * Sanitizes text for use in HTML attributes
 * @param {string} text - The text to sanitize
 * @returns {string} The sanitized text
 */
export function sanitizeHtmlAttribute(text) {
  if (!text) return '';
  
  // Remove characters that could break HTML attributes
  return text.replace(/"/g, '&quot;')
            .replace(/'/g, '&#x27;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;');
}

/**
 * Counts the number of words in a text
 * @param {string} text - The text to count words for
 * @returns {number} The number of words
 */
export function countWords(text) {
  if (!text) return 0;
  
  // Match words (sequences of word characters)
  const matches = text.match(/\b\w+\b/g);
  return matches ? matches.length : 0;
}

/**
 * Counts the number of sentences in a text
 * @param {string} text - The text to count sentences for
 * @returns {number} The number of sentences
 */
export function countSentences(text) {
  if (!text) return 0;
  
  // Match sentences ending with ., !, or ?
  const matches = text.match(/[.!?]+\s+/g);
  return matches ? matches.length : 0;
}

/**
 * Truncates text to a specified length with an ellipsis
 * @param {string} text - The text to truncate
 * @param {number} maxLength - The maximum length
 * @param {string} [suffix='...'] - The suffix to append
 * @returns {string} The truncated text
 */
export function truncateText(text, maxLength, suffix = '...') {
  if (!text) return '';
  
  if (text.length <= maxLength) {
    return text;
  }
  
  // Find the last word boundary before maxLength
  const truncated = text.substr(0, maxLength - suffix.length);
  const lastSpaceIndex = truncated.lastIndexOf(' ');
  
  if (lastSpaceIndex > 0) {
    return truncated.substr(0, lastSpaceIndex) + suffix;
  } else {
    return truncated + suffix;
  }
}

/**
 * Converts a flat array of items with parent IDs into a nested structure
 * @param {Array} items - The flat array of items
 * @param {string} [parentIdField='parentId'] - The name of the parent ID field
 * @param {string} [idField='id'] - The name of the ID field
 * @param {string} [childrenField='children'] - The name of the children array field
 * @returns {Array} The nested structure
 */
export function nestFlatArray(items, parentIdField = 'parentId', idField = 'id', childrenField = 'children') {
  if (!items || !Array.isArray(items)) {
    return [];
  }
  
  // Create a map of item ID to item
  const itemMap = {};
  items.forEach(item => {
    itemMap[item[idField]] = { ...item, [childrenField]: [] };
  });
  
  const roots = [];
  
  // Put each item in its parent's children array
  items.forEach(item => {
    const parentId = item[parentIdField];
    const itemWithChildren = itemMap[item[idField]];
    
    if (parentId === null || parentId === undefined || parentId === '') {
      // Root-level item
      roots.push(itemWithChildren);
    } else {
      // Child item
      const parent = itemMap[parentId];
      if (parent) {
        parent[childrenField].push(itemWithChildren);
      }
    }
  });
  
  return roots;
}

/**
 * Flattens a nested structure into a flat array
 * @param {Array} nestedItems - The nested array of items
 * @param {string} [childrenField='children'] - The name of the children array field
 * @returns {Array} The flat array
 */
export function flattenNestedArray(nestedItems, childrenField = 'children') {
  if (!nestedItems || !Array.isArray(nestedItems)) {
    return [];
  }
  
  const flatArray = [];
  
  function flattenRecursive(items) {
    for (const item of items) {
      // Create a copy of the item without its children
      const itemWithoutChildren = { ...item };
      delete itemWithoutChildren[childrenField];
      
      flatArray.push(itemWithoutChildren);
      
      // Recursively flatten children if they exist
      if (item[childrenField] && Array.isArray(item[childrenField])) {
        flattenRecursive(item[childrenField]);
      }
    }
  }
  
  flattenRecursive(nestedItems);
  
  return flatArray;
}

/**
 * Creates a unique identifier
 * @param {string} [prefix='id'] - Optional prefix for the ID
 * @returns {string} The unique identifier
 */
export function createId(prefix = 'id') {
  // Generate a random string based on current time and random number
  return `${prefix}_${Date.now()}_${Math.floor(Math.random() * 1000000)}`;
}

/**
 * Formats a number with thousands separators
 * @param {number} num - The number to format
 * @param {number} [decimals=0] - Number of decimal places to show
 * @returns {string} The formatted number
 */
export function formatNumber(num, decimals = 0) {
  if (typeof num !== 'number') {
    return String(num);
  }
  
  return num.toLocaleString(undefined, {
    minimumFractionDigits: decimals,
    maximumFractionDigits: decimals
  });
}

/**
 * Formats a large number in a human-readable form (e.g., 1000 -> 1K)
 * @param {number} num - The number to format
 * @returns {string} The formatted number
 */
export function formatLargeNumber(num) {
  if (typeof num !== 'number') return String(num);
  
  if (num >= 1000000) {
    return (num / 1000000).toFixed(1) + 'M';
  } else if (num >= 1000) {
    return (num / 1000).toFixed(1) + 'K';
  } else {
    return num.toString();
  }
}

/**
 * Formats a file size in bytes to a human-readable form
 * @param {number} bytes - The file size in bytes
 * @returns {string} The formatted file size
 */
export function formatFileSize(bytes) {
  if (bytes === 0) return '0 Bytes';
  
  const k = 1024;
  const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB'];
  const i = Math.floor(Math.log(bytes) / Math.log(k));
  
  return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
}

/**
 * Sanitizes a filename by removing or replacing invalid characters
 * @param {string} filename - The filename to sanitize
 * @returns {string} The sanitized filename
 */
export function sanitizeFilename(filename) {
  if (!filename) return '';
  
  // Remove or replace invalid characters for filenames
  return filename
    .replace(/[<>:"/\\|?*]/g, '_') // Replace invalid chars with underscore
    .replace(/[\s]+/g, '_') // Replace spaces with underscores
    .replace(/_{2,}/g, '_') // Replace multiple underscores with single
    .substring(0, 255); // Limit length
}

/**
 * Converts an object to query string parameters
 * @param {Object} params - The parameters object
 * @returns {string} The query string
 */
export function objectToQueryString(params) {
  if (!params) return '';
  
  const keyValuePairs = [];
  
  for (const key in params) {
    if (Object.prototype.hasOwnProperty.call(params, key)) {
      if (Array.isArray(params[key])) {
        params[key].forEach(value => {
          keyValuePairs.push(`${encodeURIComponent(key)}=${encodeURIComponent(value)}`);
        });
      } else {
        keyValuePairs.push(`${encodeURIComponent(key)}=${encodeURIComponent(params[key])}`);
      }
    }
  }
  
  return keyValuePairs.join('&');
}

export default {
  formatDate,
  formatAsFilename,
  camelToKebab,
  toTitleCase,
  sanitizeHtmlAttribute,
  countWords,
  countSentences,
  truncateText,
  nestFlatArray,
  flattenNestedArray,
  createId,
  formatNumber,
  formatLargeNumber,
  formatFileSize,
  sanitizeFilename,
  objectToQueryString
};