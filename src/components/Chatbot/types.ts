// Type definitions for the chatbot component based on the data model

export interface UserQuery {
  query_text: string;
  selected_text?: string;
  query_id: string;
  timestamp: Date;
  page_context: {
    url: string;
    title: string;
    section: string;
  };
}

export interface SourceChunk {
  chunk_id: string;
  text: string;
  url: string;
  page_title: string;
  section: string;
  chunk_index: number;
  source_url: string;
  chunk_hash: string;
  score: number;
}

export interface ChatbotResponse {
  response_id: string;
  query_id: string;
  answer_text: string;
  source_chunks: SourceChunk[];
  timestamp: Date;
  status: 'success' | 'error' | 'empty';
  error_message?: string;
}

export interface ChatbotState {
  state_type: 'idle' | 'loading' | 'error' | 'success' | 'empty';
  current_query?: UserQuery;
  current_response?: ChatbotResponse;
  error_details?: {
    message: string;
    code: string;
    timestamp: Date;
  };
}

export interface SelectedTextContext {
  text: string;
  start_position: number;
  end_position: number;
  element_id?: string;
  page_url: string;
  timestamp: Date;
}

// API request/response types
export interface QueryRequest {
  query: string;
  selected_text?: string;
  context?: {
    page_url: string;
    page_title: string;
    section: string;
  };
  top_k?: number;
}

export interface QueryResponse {
  response_id: string;
  query_id: string;
  answer_text: string;
  source_chunks: SourceChunk[];
  status: 'success' | 'error' | 'empty';
  timestamp: string;
}

export interface ErrorResponse {
  response_id: string;
  query_id?: string;
  status: 'error';
  error_message: string;
  error_code: string;
  timestamp: string;
}