/**
 * AssessmentLogic module
 * Contains business logic for assessments including grading, progress tracking, etc.
 */

/**
 * Function to grade an assessment response
 * @param {Object} assessment - The assessment object
 * @param {number} selectedAnswer - The selected answer index
 * @returns {Object} Grading result with correctness and explanation
 */
export function gradeAssessment(assessment, selectedAnswer) {
  const isCorrect = selectedAnswer === assessment.correctAnswer;
  const explanation = isCorrect 
    ? assessment.explanation 
    : assessment.incorrectExplanations[selectedAnswer] || assessment.explanation;
  
  return {
    isCorrect,
    explanation,
    correctAnswer: assessment.correctAnswer
  };
}

/**
 * Function to calculate assessment score
 * @param {Array} responses - Array of response objects
 * @returns {Object} Score results
 */
export function calculateScore(responses) {
  if (!responses || responses.length === 0) {
    return {
      score: 0,
      percentage: 0,
      total: 0,
      correct: 0
    };
  }

  const correctCount = responses.filter(r => r.isCorrect).length;
  const percentage = responses.length > 0 
    ? Math.round((correctCount / responses.length) * 100) 
    : 0;

  return {
    score: correctCount,
    percentage,
    total: responses.length,
    correct: correctCount
  };
}

/**
 * Function to check if an assessment is complete
 * @param {Array} responses - Array of response objects
 * @param {number} totalQuestions - Total number of questions in the assessment
 * @returns {boolean} Whether the assessment is complete
 */
export function isAssessmentComplete(responses, totalQuestions) {
  return responses && responses.length === totalQuestions;
}

/**
 * Function to get assessment statistics
 * @param {Array} responses - Array of response objects
 * @returns {Object} Statistics about the assessment
 */
export function getAssessmentStats(responses) {
  if (!responses || responses.length === 0) {
    return {
      total: 0,
      correct: 0,
      incorrect: 0,
      avgTimePerQuestion: 0
    };
  }

  const correctCount = responses.filter(r => r.isCorrect).length;
  const incorrectCount = responses.length - correctCount;

  // Calculate average time per question if time data exists
  let avgTimePerQuestion = 0;
  if (responses[0] && responses[0].timeSpent) {
    const totalTime = responses.reduce((sum, r) => sum + (r.timeSpent || 0), 0);
    avgTimePerQuestion = Math.round(totalTime / responses.length);
  }

  return {
    total: responses.length,
    correct: correctCount,
    incorrect: incorrectCount,
    avgTimePerQuestion
  };
}

/**
 * Function to validate assessment data
 * @param {Object} assessment - The assessment object to validate
 * @returns {Array} Array of validation errors
 */
export function validateAssessment(assessment) {
  const errors = [];

  if (!assessment.id) {
    errors.push('Assessment ID is required');
  }

  if (!assessment.questionText) {
    errors.push('Question text is required');
  }

  if (!assessment.options || assessment.options.length < 2) {
    errors.push('At least 2 answer options are required');
  }

  if (assessment.correctAnswer === undefined || assessment.correctAnswer === null) {
    errors.push('Correct answer index is required');
  } else if (assessment.correctAnswer < 0 || assessment.correctAnswer >= assessment.options.length) {
    errors.push('Correct answer index is out of range');
  }

  if (!assessment.explanation) {
    errors.push('Explanation for correct answer is required');
  }

  return errors;
}