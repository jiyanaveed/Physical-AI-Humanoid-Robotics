import React, { useState, useEffect, useRef } from 'react';
import { QueryRequest, QueryResponse, ErrorResponse } from './types';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import ragApiClient from '@site/src/utils/ragApiClient';
import styles from './Chatbot.module.css';

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  sources?: any;
}

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedTextContent = window.getSelection()?.toString().trim();
      if (selectedTextContent) {
        setSelectedText(selectedTextContent);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const closeChat = () => {
    setIsOpen(false);
  };

  const handleSendMessage = async (message: string, contextText?: string) => {
    // Add user message to chat
    const userMessage: Message = {
      id: `msg_${Date.now()}`,
      text: message,
      isUser: true
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Prepare the query request
      const queryRequest: QueryRequest = {
        query: message,
        selected_text: contextText || undefined,
        context: {
          page_url: window.location.href,
          page_title: document.title,
          section: '' // Could extract from current page context
        },
        top_k: 5
      };

      // Send request to RAG API
      const response = await ragApiClient.post<QueryResponse | ErrorResponse>('/query', queryRequest);

      // Check if it's an error response
      if ('error_message' in response.data) {
        const errorResponse = response.data as ErrorResponse;
        setError(errorResponse.error_message);
        setIsLoading(false);
        return;
      }

      // Process successful response
      const successResponse = response.data as QueryResponse;

      // Create bot message with sources
      const botMessage: Message = {
        id: `msg_${Date.now()}`,
        text: successResponse.answer_text,
        isUser: false,
        sources: successResponse.source_chunks
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (err: any) {
      console.error('Error querying RAG API:', err);
      setError(err.message || 'An error occurred while processing your request');
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearSelectedText = () => {
    setSelectedText('');
  };

  return (
    <div className={styles.chatbotContainer}>
      {!isOpen ? (
        <button
          className={styles.toggleButton}
          onClick={toggleChat}
          aria-label="Open chatbot"
        >
          💬
        </button>
      ) : (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3 className={styles.chatTitle}>Book Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={closeChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <ChatMessage
                key={message.id}
                text={message.text}
                isUser={message.isUser}
                sources={message.sources}
              />
            ))}

            {isLoading && (
              <div className={styles.loadingIndicator}>
                <span>Thinking...</span>
              </div>
            )}

            {error && (
              <div className={styles.errorContainer}>
                Error: {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <ChatInput
            onSendMessage={handleSendMessage}
            isLoading={isLoading}
            selectedText={selectedText}
            onClearSelectedText={handleClearSelectedText}
          />
        </div>
      )}
    </div>
  );
};

export default Chatbot;