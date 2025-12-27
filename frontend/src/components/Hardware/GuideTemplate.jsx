import React from 'react';
import clsx from 'clsx';
import styles from './GuideTemplate.module.css';

/**
 * Properties for the GuideTemplate component
 * @typedef {Object} HardwareGuideProps
 * @property {string} id - Unique identifier for the hardware profile
 * @property {string} name - Descriptive name of the profile
 * @property {string} cpuRequirements - Description of CPU requirements
 * @property {string} gpuRequirements - Description of GPU requirements (optional)
 * @property {string} memoryRequirements - Amount of RAM required
 * @property {string} storageRequirements - Amount of disk space required
 * @property {string} networkRequirements - Network requirements if any
 * @property {Object} performanceBenchmarks - Reference hardware with benchmark times
 * @property {string[]} tags - Array of tags for categorizing the exercises that use this profile
 */

/**
 * GuideTemplate component for displaying hardware requirements
 * @param {HardwareGuideProps} props - Component properties
 * @returns {JSX.Element} The hardware guide component
 */
function GuideTemplate(props) {
  const {
    name,
    cpuRequirements,
    gpuRequirements,
    memoryRequirements,
    storageRequirements,
    networkRequirements,
    performanceBenchmarks,
    tags = []
  } = props;

  return (
    <div className={clsx('card', styles.hardwareCard)}>
      <div className="card__header">
        <h2>Hardware Requirements: {name}</h2>
      </div>
      
      <div className="card__body">
        <div className={styles.requirementsGrid}>
          <div className={styles.requirementItem}>
            <h3>CPU</h3>
            <p>{cpuRequirements}</p>
          </div>
          
          {gpuRequirements && (
            <div className={styles.requirementItem}>
              <h3>GPU</h3>
              <p>{gpuRequirements}</p>
            </div>
          )}
          
          <div className={styles.requirementItem}>
            <h3>Memory</h3>
            <p>{memoryRequirements}</p>
          </div>
          
          <div className={styles.requirementItem}>
            <h3>Storage</h3>
            <p>{storageRequirements}</p>
          </div>
          
          {networkRequirements && (
            <div className={styles.requirementItem}>
              <h3>Network</h3>
              <p>{networkRequirements}</p>
            </div>
          )}
        </div>
        
        {performanceBenchmarks && (
          <section className={styles.section}>
            <h3>Performance Benchmarks</h3>
            <div className={styles.benchmarkTable}>
              <table>
                <thead>
                  <tr>
                    <th>Hardware</th>
                    <th>Benchmark Task</th>
                    <th>Duration</th>
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <td>{performanceBenchmarks.hardware}</td>
                    {performanceBenchmarks.benchmarks.map((benchmark, index) => (
                      <tr key={index}>
                        <td>{benchmark.task}</td>
                        <td>{benchmark.duration}s</td>
                      </tr>
                    ))}
                  </tr>
                </tbody>
              </table>
            </div>
          </section>
        )}
      </div>
      
      {tags.length > 0 && (
        <div className="card__footer">
          {tags.map((tag, index) => (
            <span key={index} className="badge badge--info margin-right--sm">
              {tag}
            </span>
          ))}
        </div>
      )}
    </div>
  );
}

export default GuideTemplate;