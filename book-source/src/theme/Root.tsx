/**
 * Docusaurus Root wrapper component
 * Wraps the entire application to add global functionality
 * Docs: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React, { useState } from 'react';
import { ChatInterface } from '../components/ChatInterface';
import styles from './Root.module.css';

export default function Root({ children }: { children: React.ReactNode }) {
  const [isChatOpen, setIsChatOpen] = useState(false);

  const openChat = () => setIsChatOpen(true);
  const closeChat = () => setIsChatOpen(false);

  return (
    <>
      {children}

      {/* Floating chat button */}
      {!isChatOpen && (
        <button
          onClick={openChat}
          className={styles.floatingChatButton}
          aria-label="Open AI Assistant"
          title="Ask questions about Physical AI & Humanoid Robotics"
        >
          <span className={styles.chatIcon}>💬</span>
          <span className={styles.chatLabel}>AI Assistant</span>
        </button>
      )}

      {/* Chat interface overlay */}
      <ChatInterface isOpen={isChatOpen} onClose={closeChat} />
    </>
  );
}
