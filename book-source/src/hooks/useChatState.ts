/**
 * React hook for managing chat state and API interactions
 * Handles sending messages, loading states, and errors
 */

import { useState, useCallback } from 'react';
import type { Message, ChatError, ChatQueryRequest } from '../types/chat';
import { sendChatQuery, apiErrorToChatError } from '../services/chatApi';
import { useConversationHistory } from './useConversationHistory';

interface UseChatStateReturn {
  isLoading: boolean;
  error: ChatError | null;
  sendMessage: (query: string, selectedText?: string | null) => Promise<void>;
  clearError: () => void;
}

/**
 * Hook for managing chat state
 */
export function useChatState(): UseChatStateReturn {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<ChatError | null>(null);

  const {
    conversation,
    addMessage,
    getRecentMessages,
    hasActiveConversation,
  } = useConversationHistory();

  /**
   * Send chat message to backend
   */
  const sendMessage = useCallback(
    async (query: string, selectedText?: string | null) => {
      // Clear previous error
      setError(null);
      setIsLoading(true);

      try {
        // Add user message to conversation
        const userMessage: Message = {
          role: 'user',
          content: query,
          timestamp: new Date().toISOString(),
          selected_text: selectedText || null,
        };
        addMessage(userMessage);

        // Prepare API request
        const request: ChatQueryRequest = {
          query,
          selected_text: selectedText || null,
          conversation_history: getRecentMessages(5), // Last 5 messages before this one
          conversation_id: hasActiveConversation()
            ? conversation?.conversation_id || null
            : null,
          conversation_created_at: hasActiveConversation()
            ? conversation?.created_at || null
            : null,
        };

        // Send to backend
        const response = await sendChatQuery(request);

        // Add assistant response to conversation
        const assistantMessage: Message = {
          role: 'assistant',
          content: response.response,
          timestamp: response.timestamp,
          sources: response.sources,
        };
        addMessage(assistantMessage);
      } catch (err) {
        // Handle error
        const chatError = apiErrorToChatError(err);
        setError(chatError);

        // Also add error message to conversation for context
        const errorMessage: Message = {
          role: 'assistant',
          content: `Error: ${chatError.error}${
            chatError.detail ? ` - ${chatError.detail}` : ''
          }`,
          timestamp: new Date().toISOString(),
        };
        addMessage(errorMessage);
      } finally {
        setIsLoading(false);
      }
    },
    [conversation, addMessage, getRecentMessages, hasActiveConversation]
  );

  /**
   * Clear error state
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  return {
    isLoading,
    error,
    sendMessage,
    clearError,
  };
}
