/**
 * React hook for managing conversation history in localStorage
 * Handles 7-day expiry and automatic cleanup
 */

import { useState, useEffect, useCallback } from 'react';
import type { Conversation, Message } from '../types/chat';
import { CHAT_CONSTANTS } from '../types/chat';

/**
 * Generate new conversation ID
 */
function generateConversationId(): string {
  return `conv_${Date.now()}_${Math.random().toString(36).substring(2, 9)}`;
}

/**
 * Calculate expiry date (7 days from now)
 */
function calculateExpiryDate(createdAt: Date): string {
  const expiry = new Date(createdAt);
  expiry.setDate(expiry.getDate() + CHAT_CONSTANTS.CONVERSATION_EXPIRY_DAYS);
  return expiry.toISOString();
}

/**
 * Check if conversation has expired
 */
function isConversationExpired(expiresAt: string): boolean {
  return new Date() > new Date(expiresAt);
}

/**
 * Load conversation from localStorage
 */
function loadConversation(): Conversation | null {
  if (typeof window === 'undefined') return null;

  try {
    const stored = localStorage.getItem(CHAT_CONSTANTS.LOCALSTORAGE_KEY);
    if (!stored) return null;

    const conversation: Conversation = JSON.parse(stored);

    // Check expiry
    if (isConversationExpired(conversation.expires_at)) {
      console.log('Conversation expired, clearing...');
      localStorage.removeItem(CHAT_CONSTANTS.LOCALSTORAGE_KEY);
      return null;
    }

    return conversation;
  } catch (error) {
    console.error('Failed to load conversation from localStorage:', error);
    return null;
  }
}

/**
 * Save conversation to localStorage
 */
function saveConversation(conversation: Conversation): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(
      CHAT_CONSTANTS.LOCALSTORAGE_KEY,
      JSON.stringify(conversation)
    );
  } catch (error) {
    console.error('Failed to save conversation to localStorage:', error);
  }
}

/**
 * Hook for managing conversation history
 */
export function useConversationHistory() {
  const [conversation, setConversation] = useState<Conversation | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Load conversation on mount
  useEffect(() => {
    const loaded = loadConversation();
    setConversation(loaded);
    setIsLoading(false);
  }, []);

  // Save conversation whenever it changes
  useEffect(() => {
    if (conversation) {
      saveConversation(conversation);
    }
  }, [conversation]);

  /**
   * Start new conversation
   */
  const startNewConversation = useCallback(() => {
    const now = new Date();
    const newConversation: Conversation = {
      conversation_id: generateConversationId(),
      messages: [],
      created_at: now.toISOString(),
      expires_at: calculateExpiryDate(now),
      last_updated: now.toISOString(),
    };

    setConversation(newConversation);
    return newConversation;
  }, []);

  /**
   * Add message to conversation
   */
  const addMessage = useCallback((message: Message) => {
    setConversation((prev) => {
      // Create new conversation if none exists
      if (!prev) {
        const newConv = startNewConversation();
        return {
          ...newConv,
          messages: [message],
          last_updated: new Date().toISOString(),
        };
      }

      // Add message to existing conversation
      return {
        ...prev,
        messages: [...prev.messages, message],
        last_updated: new Date().toISOString(),
      };
    });
  }, [startNewConversation]);

  /**
   * Clear conversation (reset)
   */
  const clearConversation = useCallback(() => {
    if (typeof window !== 'undefined') {
      localStorage.removeItem(CHAT_CONSTANTS.LOCALSTORAGE_KEY);
    }
    setConversation(null);
  }, []);

  /**
   * Get last N messages for API request
   */
  const getRecentMessages = useCallback((limit: number = CHAT_CONSTANTS.MAX_CONVERSATION_HISTORY): Message[] => {
    if (!conversation) return [];
    return conversation.messages.slice(-limit);
  }, [conversation]);

  /**
   * Check if conversation exists and is valid
   */
  const hasActiveConversation = useCallback((): boolean => {
    if (!conversation) return false;
    return !isConversationExpired(conversation.expires_at);
  }, [conversation]);

  return {
    conversation,
    isLoading,
    startNewConversation,
    addMessage,
    clearConversation,
    getRecentMessages,
    hasActiveConversation,
  };
}
