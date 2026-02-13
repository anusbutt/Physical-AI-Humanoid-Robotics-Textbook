/**
 * Docusaurus Root wrapper component
 * Wraps the entire application to add global functionality
 * Docs: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React, { useState, useRef } from 'react';
import { ChatInterface } from '../components/ChatInterface';
import styles from './Root.module.css';

export default function Root({ children }: { children: React.ReactNode }) {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [initialSelectedText, setInitialSelectedText] = useState<string>('');
  // Ref to capture selection on mousedown (before browser clears it on click)
  const capturedSelectionRef = useRef<string>('');

  const handleMouseDown = () => {
    // Browser clears selection on mousedown, so capture it HERE
    const selection = window.getSelection();
    capturedSelectionRef.current = selection ? selection.toString().trim() : '';
  };

  const openChat = () => {
    // Use the text captured during mousedown (selection is already gone by onClick)
    setInitialSelectedText(capturedSelectionRef.current);
    capturedSelectionRef.current = '';
    setIsChatOpen(true);
  };

  const closeChat = () => {
    setIsChatOpen(false);
    setInitialSelectedText('');
  };

  return (
    <>
      {children}

      {/* Floating chat button */}
      {!isChatOpen && (
        <button
          onMouseDown={handleMouseDown}
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
      <ChatInterface
        isOpen={isChatOpen}
        onClose={closeChat}
        initialSelectedText={initialSelectedText}
      />
    </>
  );
}
