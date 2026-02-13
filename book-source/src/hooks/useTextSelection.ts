/**
 * React hook for detecting and managing text selection on the page
 * Used for "Explain selected text" feature
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { CHAT_CONSTANTS } from '../types/chat';

/**
 * Get currently selected text from window
 */
function getSelectedText(): string {
  if (typeof window === 'undefined') return '';

  const selection = window.getSelection();
  if (!selection) return '';

  const text = selection.toString().trim();
  return text;
}

/**
 * Validate selected text (10-2000 characters)
 */
function isValidSelection(text: string): boolean {
  const length = text.length;
  return (
    length >= CHAT_CONSTANTS.MIN_SELECTED_TEXT_LENGTH &&
    length <= CHAT_CONSTANTS.MAX_SELECTED_TEXT_LENGTH
  );
}

/**
 * Hook for managing text selection
 * @param initialText - Optional text captured before the hook mounted (e.g. from Root.tsx on chat open)
 */
export function useTextSelection(initialText?: string) {
  const [selectedText, setSelectedText] = useState<string>(initialText || '');
  const [hasValidSelection, setHasValidSelection] = useState<boolean>(
    isValidSelection(initialText || '')
  );
  // Track whether we're using an initialText that shouldn't be overwritten by empty browser selection
  const usingInitialTextRef = useRef<boolean>(isValidSelection(initialText || ''));

  /**
   * Update selection state from browser
   */
  const updateSelection = useCallback(() => {
    const text = getSelectedText();

    // If browser selection is empty but we have initialText, don't overwrite
    if (!text && usingInitialTextRef.current) {
      return;
    }

    // New browser selection made — stop protecting initialText
    if (text) {
      usingInitialTextRef.current = false;
    }

    setSelectedText(text);
    setHasValidSelection(isValidSelection(text));
  }, []);

  /**
   * Clear selection
   */
  const clearSelection = useCallback(() => {
    usingInitialTextRef.current = false;
    setSelectedText('');
    setHasValidSelection(false);

    // Also clear browser selection
    if (typeof window !== 'undefined') {
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
      }
    }
  }, []);

  /**
   * Sync when initialText changes (e.g. chat re-opened with new selection)
   */
  useEffect(() => {
    if (initialText && isValidSelection(initialText)) {
      usingInitialTextRef.current = true;
      setSelectedText(initialText);
      setHasValidSelection(true);
    } else if (!initialText) {
      usingInitialTextRef.current = false;
    }
  }, [initialText]);

  /**
   * Listen for selection changes
   */
  useEffect(() => {
    if (typeof window === 'undefined') return;

    // Update on mouseup (after selection is made)
    const handleMouseUp = () => {
      // Small delay to ensure selection is complete
      setTimeout(updateSelection, 10);
    };

    // Update on keyup (for keyboard selection)
    const handleKeyUp = (e: KeyboardEvent) => {
      // Only update if it's a selection-related key
      if (e.shiftKey || e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
        setTimeout(updateSelection, 10);
      }
    };

    window.addEventListener('mouseup', handleMouseUp);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('mouseup', handleMouseUp);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [updateSelection]);

  return {
    selectedText,
    hasValidSelection,
    clearSelection,
    updateSelection,
  };
}
