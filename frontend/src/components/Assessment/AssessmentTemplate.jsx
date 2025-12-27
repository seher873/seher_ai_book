import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './AssessmentTemplate.module.css';

/**
 * Properties for the AssessmentTemplate component
 * @typedef {Object} AssessmentProps
 * @property {string} id - Unique identifier for the assessment
 * @property {string} moduleId - Reference to the module/chapter this assessment relates to
 * @property {string} questionText - The actual question text
 * @property {string} questionType - Type of question, defaults to "multiple-choice"
 * @property {string[]} options - Array of possible answer options
 * @property {number} correctAnswer - Index of the correct answer option
 * @property {string} explanation - Detailed explanation of the correct answer
 * @property {string[]} incorrectExplanations - Array of explanations for incorrect options
 * @property {string} difficultyLevel - Level of difficulty (beginner, intermediate, advanced)
 * @property {string[]} tags - Array of tags for categorizing the question
 */

/**
 * AssessmentRenderer component for displaying interactive assessments with enhanced functionality
 * @param {AssessmentProps} props - Component properties
 * @returns {JSX.Element} The assessment component
 */
function AssessmentRenderer(props) {
  const [selectedOption, setSelectedOption] = useState(null);
  const [submitted, setSubmitted] = useState(false);
  const [showExplanation, setShowExplanation] = useState(false);
  const [isCorrect, setIsCorrect] = useState(false);
  const [attemptCount, setAttemptCount] = useState(0);

  const handleOptionChange = (index) => {
    if (!submitted) {
      setSelectedOption(index);
    }
  };

  const handleSubmit = () => {
    setSubmitted(true);
    setShowExplanation(true);
    setIsCorrect(selectedOption === props.correctAnswer);
    setAttemptCount(attemptCount + 1);
  };

  const handleReset = () => {
    setSelectedOption(null);
    setSubmitted(false);
    setShowExplanation(false);
    setIsCorrect(false);
  };

  // Enhanced feedback based on attempts and correctness
  const getFeedbackMessage = () => {
    if (!submitted) return '';
    if (isCorrect) {
      return attemptCount === 1 ? 'Excellent! You got it right on the first try.' : 'Correct! Well done.';
    } else {
      return attemptCount === 1 ? 'Not quite right. Review the explanation and try again.' : 'Still incorrect. Study the explanation carefully.';
    }
  };

  return (
    <div className={clsx('card', styles.assessmentCard)}>
      <div className="card__header">
        <h3>Knowledge Check</h3>
        <div className={styles.headerBadges}>
          <span className={clsx('badge', `badge--${props.difficultyLevel}`)}>
            {props.difficultyLevel}
          </span>
          {attemptCount > 0 && (
            <span className={clsx('badge', isCorrect ? 'badge--success' : 'badge--danger')}>
              {isCorrect ? 'Correct' : 'Incorrect'} - Attempt {attemptCount}
            </span>
          )}
        </div>
      </div>
      <div className="card__body">
        <p>{props.questionText}</p>
        <div className={styles.optionsContainer}>
          {props.options.map((option, index) => (
            <div key={index} className={styles.optionItem}>
              <label
                className={clsx(
                  styles.optionLabel,
                  submitted && index === props.correctAnswer ? styles.correctOption : '',
                  submitted && index === selectedOption && !isCorrect ? styles.incorrectOption : ''
                )}
              >
                <input
                  type="radio"
                  name={`assessment-${props.id}`}
                  checked={selectedOption === index}
                  onChange={() => handleOptionChange(index)}
                  disabled={submitted}
                  className={styles.optionInput}
                />
                <span className={styles.optionText}>{option}</span>
              </label>
            </div>
          ))}
        </div>

        {!submitted ? (
          <button
            className={clsx('button button--primary', styles.submitButton)}
            onClick={handleSubmit}
            disabled={selectedOption === null}
          >
            Submit Answer
          </button>
        ) : (
          <div className={styles.feedbackContainer}>
            <div className={clsx(
              'alert',
              isCorrect ? 'alert--success' : 'alert--danger'
            )}>
              <p><strong>{getFeedbackMessage()}</strong></p>

              {(isCorrect || showExplanation) && (
                <p>
                  {isCorrect
                    ? props.explanation
                    : props.incorrectExplanations[selectedOption] || props.explanation}
                </p>
              )}
            </div>
            <button
              className={clsx('button button--secondary', styles.resetButton)}
              onClick={handleReset}
            >
              Try Again
            </button>
          </div>
        )}
      </div>
      {props.tags && props.tags.length > 0 && (
        <div className="card__footer">
          {props.tags.map((tag, index) => (
            <span key={index} className="badge badge--info margin-right--sm">
              {tag}
            </span>
          ))}
        </div>
      )}
    </div>
  );
}

export default AssessmentRenderer;