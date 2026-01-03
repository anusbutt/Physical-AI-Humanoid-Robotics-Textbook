/**
 * ChatInterface component - Main chat interface container
 * Brings together all chat components and manages state
 */

import React, { useRef, useEffect } from 'react';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import { useConversationHistory } from '../hooks/useConversationHistory';
import { useChatState } from '../hooks/useChatState';
import { useTextSelection } from '../hooks/useTextSelection';
import styles from './ChatInterface.module.css';

interface ChatInterfaceProps {
  isOpen: boolean;
  onClose: () => void;
}

export const ChatInterface: React.FC<ChatInterfaceProps> = ({ isOpen, onClose }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const {
    conversation,
    clearConversation,
  } = useConversationHistory();

  const { isLoading, error, sendMessage, clearError } = useChatState();
  const { selectedText, hasValidSelection, clearSelection } = useTextSelection();

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

  const handleSendMessage = async (query: string, selectedText?: string | null) => {
    await sendMessage(query, selectedText);
  };

  const handleNewConversation = () => {
    if (window.confirm('Start a new conversation? Current conversation will be cleared.')) {
      clearConversation();
      clearSelection();
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.overlay}>
      <div className={styles.chatContainer}>
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerTitle}>
            <h2>AI Assistant</h2>
            <p className={styles.subtitle}>Physical AI & Humanoid Robotics</p>
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
        {error && (
          <div className={styles.errorBanner}>
            <strong>Error:</strong> {error.error}
            {error.detail && <span> - {error.detail}</span>}
            <button
              onClick={clearError}
              className={styles.errorDismiss}
              aria-label="Dismiss error"
            >
              ✕
            </button>
          </div>
        )}

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
      </div>
    </div>
  );
};
