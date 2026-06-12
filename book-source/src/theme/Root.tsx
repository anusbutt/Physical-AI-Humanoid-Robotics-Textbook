/**
 * Docusaurus Root wrapper component
 * Wraps the entire application to add global functionality
 * Docs: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React, { useState, useRef, useEffect, useCallback } from 'react';
import { ChatInterface } from '../components/ChatInterface';
import { CHAT_CONSTANTS } from '../types/chat';
import styles from './Root.module.css';

export default function Root({ children }: { children: React.ReactNode }) {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [initialSelectedText, setInitialSelectedText] = useState<string>('');
  // Ref to capture selection as soon as the user finishes selecting (mouseup)
  const capturedSelectionRef = useRef<string>('');

  // Listen for the "Try the AI Tutor" CTA from the landing page
  useEffect(() => {
    const handler = () => {
      setIsChatOpen(true);
    };
    window.addEventListener('open-chat-demo', handler);
    return () => window.removeEventListener('open-chat-demo', handler);
  }, []);

  /**
   * Capture text selection on mouseup — this fires as soon as the user
   * finishes selecting text, BEFORE they click the chat button.
   * This way the selection is still active when we read it.
   */
  useEffect(() => {
    const handleMouseUp = () => {
      // Small delay to ensure the selection is finalized
      setTimeout(() => {
        const selection = window.getSelection();
        const text = selection ? selection.toString().trim() : '';
        // Only store if it meets the minimum length requirement
        if (text.length >= CHAT_CONSTANTS.MIN_SELECTED_TEXT_LENGTH) {
          capturedSelectionRef.current = text;
        }
      }, 10);
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      if (e.shiftKey || e.key === 'ArrowLeft' || e.key === 'ArrowRight' ||
          e.key === 'ArrowUp' || e.key === 'ArrowDown') {
        setTimeout(() => {
          const selection = window.getSelection();
          const text = selection ? selection.toString().trim() : '';
          if (text.length >= CHAT_CONSTANTS.MIN_SELECTED_TEXT_LENGTH) {
            capturedSelectionRef.current = text;
          }
        }, 10);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('keyup', handleKeyUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  const openChat = useCallback(() => {
    // Use the text captured during the last selection (mouseup)
    setInitialSelectedText(capturedSelectionRef.current);
    capturedSelectionRef.current = '';
    setIsChatOpen(true);
  }, []);

  const closeChat = useCallback(() => {
    setIsChatOpen(false);
    setInitialSelectedText('');
  }, []);

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
      <ChatInterface
        isOpen={isChatOpen}
        onClose={closeChat}
        initialSelectedText={initialSelectedText}
      />
    </>
  );
}
