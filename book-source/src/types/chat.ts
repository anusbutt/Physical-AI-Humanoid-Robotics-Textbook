/**
 * TypeScript type definitions for RAG chatbot
 * Matches backend schema in backend/app/models/schemas.py
 */

/**
 * Message in conversation history
 */
export interface Message {
  role: 'user' | 'assistant';
  content: string;
  timestamp?: string;
  sources?: Source[];
  selected_text?: string | null;
}

/**
 * Source citation from RAG retrieval
 */
export interface Source {
  module: string;
  lesson: string;
  section: string;
  content: string;
  similarity_score?: number;
}

/**
 * Conversation stored in browser localStorage
 */
export interface Conversation {
  conversation_id: string;
  messages: Message[];
  created_at: string; // ISO 8601 format
  expires_at: string; // ISO 8601 format (created_at + 7 days)
  last_updated: string; // ISO 8601 format
}

/**
 * Request payload for POST /api/chat/query
 */
export interface ChatQueryRequest {
  query: string; // 1-2000 characters
  selected_text?: string | null; // Optional selected text from page
  lesson_id?: string | null; // Optional lesson filter (e.g., "module-01/lesson-02")
  conversation_history: Message[]; // Last 5 messages
  conversation_id?: string | null; // For multi-turn conversations
  conversation_created_at?: string | null; // ISO 8601 format, for expiry check
}

/**
 * Response from POST /api/chat/query
 */
export interface ChatQueryResponse {
  response: string; // AI-generated answer
  sources: Source[]; // Source citations
  conversation_id: string; // Unique conversation ID
  timestamp: string; // ISO 8601 format
  retrieved_chunks?: number | null; // Number of chunks retrieved
  used_selected_text?: boolean | null; // Whether selected text was used
}

/**
 * Error response from API
 */
export interface ChatError {
  error: string;
  detail?: string;
  type?: 'rate_limit' | 'service_error' | 'validation_error' | 'filter_error' | 'unknown';
  status_code?: number;
}

/**
 * Health check response from GET /api/health
 */
export interface HealthCheckResponse {
  status: 'healthy' | 'unhealthy';
  timestamp: string;
  version: string;
  services: {
    api: boolean;
  };
}

/**
 * UI state for chat interface
 */
export interface ChatUIState {
  isLoading: boolean;
  error: ChatError | null;
  currentQuery: string;
  selectedText: string | null;
  conversation: Conversation | null;
}

/**
 * Constants
 */
export const CHAT_CONSTANTS = {
  MAX_QUERY_LENGTH: 2000,
  MIN_QUERY_LENGTH: 1,
  MAX_SELECTED_TEXT_LENGTH: 2000,
  MIN_SELECTED_TEXT_LENGTH: 10,
  MAX_CONVERSATION_HISTORY: 5, // Only last 5 messages
  CONVERSATION_EXPIRY_DAYS: 7,
  LOCALSTORAGE_KEY: 'hackathon_chat_conversation',
} as const;
