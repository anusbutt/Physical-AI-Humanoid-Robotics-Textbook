/**
 * TypingDots — reusable animated typing indicator
 * Three pulsing dots shown while the AI is "thinking"
 */

import React from 'react';
import styles from './TypingDots.module.css';

interface TypingDotsProps {
  label?: string;
  compact?: boolean;
}

export const TypingDots: React.FC<TypingDotsProps> = ({
  label = 'AI is thinking...',
  compact = false,
}) => {
  return (
    <div
      className={`${styles.dotsContainer} ${compact ? styles.compact : ''}`}
      role="status"
      aria-label={label}
      aria-live="polite"
    >
      <span className={styles.dot} />
      <span className={styles.dot} />
      <span className={styles.dot} />
      {label && <span className={styles.label}>{label}</span>}
    </div>
  );
};
