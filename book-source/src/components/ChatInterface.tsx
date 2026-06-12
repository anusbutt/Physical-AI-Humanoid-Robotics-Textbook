/**
 * ChatInterface component — Main chat interface container
 * Brings together all chat components and manages state
 * Added: enhanced animations, shared TypingDots, smooth auto-scroll
 */

import React, { useRef, useEffect } from 'react';
import { motion, AnimatePresence } from 'motion/react';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import { TypingDots } from './TypingDots';
import { useChatState } from '../hooks/useChatState';
import { useTextSelection } from '../hooks/useTextSelection';
import styles from './ChatInterface.module.css';

interface ChatInterfaceProps {
  isOpen: boolean;
  onClose: () => void;
  initialSelectedText?: string;
}

export const ChatInterface: React.FC<ChatInterfaceProps> = ({
  isOpen,
  onClose,
  initialSelectedText,
}) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const {
    conversation,
    isLoading,
    error,
    sendMessage,
    clearError,
    clearConversation,
  } = useChatState();
  const { selectedText, hasValidSelection, clearSelection } =
    useTextSelection(initialSelectedText);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [conversation?.messages]);

  // Clear error after 5 seconds
  useEffect(() => {
    if (error) {
      const timer = setTimeout(clearError, 5000);
      return () => clearTimeout(timer);
    }
  }, [error, clearError]);

  const handleSendMessage = async (
    query: string,
    selectedText?: string | null
  ) => {
    await sendMessage(query, selectedText);
  };

  const handleNewConversation = () => {
    if (
      window.confirm(
        'Start a new conversation? Current conversation will be cleared.'
      )
    ) {
      clearConversation();
      clearSelection();
    }
  };

  if (!isOpen) return null;

  return (
    <motion.div
      className={styles.overlay}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.2 }}
    >
      <motion.div
        className={styles.chatContainer}
        initial={{ transform: 'translateY(30px) scale(0.97)', opacity: 0 }}
        animate={{ transform: 'translateY(0) scale(1)', opacity: 1 }}
        exit={{ transform: 'translateY(30px) scale(0.97)', opacity: 0 }}
        transition={{ duration: 0.3, ease: [0.4, 0, 0.2, 1] as const }}
      >
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerTitle}>
            <h2>AI Assistant</h2>
            <p className={styles.subtitle}>
              Physical AI & Humanoid Robotics
            </p>
          </div>
          <div className={styles.headerActions}>
            <button
              onClick={handleNewConversation}
              className={styles.newChatButton}
              aria-label="Start new conversation"
              title="Start new conversation"
            >
              New Chat
            </button>
            <button
              onClick={onClose}
              className={styles.closeButton}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>
        </div>

        {/* Error banner */}
        <AnimatePresence>
          {error && (
            <motion.div
              className={styles.errorBanner}
              initial={{ opacity: 0, height: 0 }}
              animate={{ opacity: 1, height: 'auto' }}
              exit={{ opacity: 0, height: 0 }}
              transition={{ duration: 0.2 }}
            >
              <strong>Error:</strong> {error.error}
              {error.detail && <span> — {error.detail}</span>}
              <button
                onClick={clearError}
                className={styles.errorDismiss}
                aria-label="Dismiss error"
              >
                ✕
              </button>
            </motion.div>
          )}
        </AnimatePresence>

        {/* Selected text notice */}
        {hasValidSelection && (
          <div className={styles.selectionNotice}>
            ✨ Text selected! Ask a question to get an explanation.
          </div>
        )}

        {/* Messages area */}
        <div className={styles.messagesContainer}>
          {conversation && conversation.messages.length === 0 && (
            <div className={styles.emptyState}>
              <div className={styles.emptyStateIcon}>💬</div>
              <h3>Welcome to the AI Assistant!</h3>
              <p>Ask questions about Physical AI & Humanoid Robotics</p>
              <div className={styles.suggestions}>
                <strong>Try asking:</strong>
                <ul>
                  <li>"What is ROS2?"</li>
                  <li>"Explain sensor fusion"</li>
                  <li>"How does NVIDIA Isaac Sim work?"</li>
                  <li>Or select text on the page and ask about it!</li>
                </ul>
              </div>
            </div>
          )}

          {conversation?.messages.map((message, index) => (
            <ChatMessage key={index} message={message} />
          ))}

          {/* Typing indicator — animated shared component */}
          {isLoading && (
            <div className={styles.typingIndicator}>
              <TypingDots compact label="AI is thinking..." />
            </div>
          )}

          {/* Scroll anchor */}
          <div ref={messagesEndRef} />
        </div>

        {/* Input area */}
        <ChatInput
          onSendMessage={handleSendMessage}
          isLoading={isLoading}
          selectedText={hasValidSelection ? selectedText : null}
          onClearSelection={clearSelection}
        />

        {/* Footer */}
        <div className={styles.footer}>
          <span className={styles.footerText}>
            Conversations expire after 7 days
          </span>
          {conversation && (
            <span className={styles.conversationId}>
              ID: {conversation.conversation_id}
            </span>
          )}
        </div>
      </motion.div>
    </motion.div>
  );
};
