/**
 * ChatMessage component — Displays individual chat messages
 * Shows user queries and assistant responses with sources
 * Added: Framer Motion entrance animation
 */

import React from 'react';
import { motion } from 'motion/react';
import type { Message } from '../types/chat';
import styles from './ChatMessage.module.css';

interface ChatMessageProps {
  message: Message;
}

export const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.role === 'user';

  return (
    <motion.div
      className={`${styles.message} ${isUser ? styles.userMessage : styles.assistantMessage}`}
      initial={{ opacity: 0, y: 12 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.25, ease: [0.25, 0.46, 0.45, 0.94] as const }}
    >
      <div className={styles.messageHeader}>
        <span className={styles.role}>
          {isUser ? 'You' : 'AI Assistant'}
        </span>
        {message.timestamp && (
          <span className={styles.timestamp}>
            {new Date(message.timestamp).toLocaleTimeString()}
          </span>
        )}
      </div>

      <div className={styles.messageContent}>
        {/* Show selected text indicator for user messages */}
        {isUser && message.selected_text && (
          <div className={styles.selectedTextIndicator}>
            <strong>Selected text:</strong>
            <blockquote className={styles.selectedTextQuote}>
              {message.selected_text}
            </blockquote>
          </div>
        )}

        {/* Main message content */}
        <div className={styles.content}>
          {message.content}
        </div>

        {/* Show sources for assistant messages */}
        {!isUser && message.sources && message.sources.length > 0 && (
          <div className={styles.sources}>
            <strong className={styles.sourcesTitle}>Sources:</strong>
            <ul className={styles.sourcesList}>
              {message.sources.map((source, index) => (
                <li key={index} className={styles.sourceItem}>
                  <span className={styles.sourceLocation}>
                    {source.module} → {source.lesson}
                    {source.section ? ` → ${source.section}` : ''}
                  </span>
                  {source.similarity_score && (
                    <span className={styles.similarityScore}>
                      ({(source.similarity_score * 100).toFixed(1)}% match)
                    </span>
                  )}
                </li>
              ))}
            </ul>
          </div>
        )}
      </div>
    </motion.div>
  );
};
