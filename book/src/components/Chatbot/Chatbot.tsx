import React, { useState, useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import './Chatbot.module.css';

interface Source {
  title: string;
  url?: string;
  content: string;
}

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  citations?: string[];
  sources?: Source[];
}

// Define the API response structure
interface RAGResponse {
  response: string;
  citations?: string[];
  sources?: Array<{
    title: string;
    url?: string;
    content: string;
  }>;
}

const Chatbot: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [showContextMenu, setShowContextMenu] = useState(false);
  const [contextMenuPosition, setContextMenuPosition] = useState({ x: 0, y: 0 });
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);
  const contextMenuRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle text selection and context menu
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        setSelectedText(selection.toString());
      } else {
        setSelectedText('');
        setShowContextMenu(false);
      }
    };

    const handleContextMenu = (e: MouseEvent) => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        e.preventDefault();
        setContextMenuPosition({ x: e.clientX, y: e.clientY });
        setShowContextMenu(true);
      } else {
        setShowContextMenu(false);
      }
    };

    // Close context menu when clicking elsewhere
    const handleClickOutside = (e: MouseEvent) => {
      if (contextMenuRef.current && !contextMenuRef.current.contains(e.target as Node)) {
        setShowContextMenu(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);
    document.addEventListener('contextmenu', handleContextMenu);
    document.addEventListener('click', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
      document.removeEventListener('contextmenu', handleContextMenu);
      document.removeEventListener('click', handleClickOutside);
    };
  }, []);

  // Close context menu when chat is opened
  useEffect(() => {
    if (isOpen) {
      setShowContextMenu(false);
    }
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Function to call the RAG API
  const callRAGAPI = async (query: string): Promise<RAGResponse> => {
    // In a real implementation, this would be your actual API endpoint
    const API_URL = process.env.RAG_API_URL || 'http://localhost:8000/query';

    try {
      const response = await fetch(API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: query,
          context: selectedText, // Include selected text as context if available
        }),
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data: RAGResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error calling RAG API:', error);
      // Return a simulated response in case of error
      return {
        response: 'Sorry, I encountered an error processing your request. Please try again.',
        citations: ['API Error'],
      };
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim()) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the RAG API
      const apiResponse = await callRAGAPI(inputValue);

      // Only add the message if we got a valid response
      if (apiResponse.response) {
        const botMessage: Message = {
          id: (Date.now() + 1).toString(),
          text: apiResponse.response,
          sender: 'bot',
          timestamp: new Date(),
          citations: apiResponse.citations,
          sources: apiResponse.sources,
        };

        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage: Message = {
          id: (Date.now() + 1).toString(),
          text: 'Sorry, I couldn\'t generate a response for your query. Please try rephrasing your question.',
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error processing your request. Please check your connection and try again.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleAskAboutSelection = () => {
    if (selectedText) {
      setInputValue(`About this: ${selectedText}`);
      setIsOpen(true);
      setShowContextMenu(false);
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
      }
    }
  };

  const handleAskClarifyingQuestion = () => {
    if (selectedText) {
      setInputValue(`Can you explain this in simpler terms: ${selectedText}`);
      setIsOpen(true);
      setShowContextMenu(false);
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
      }
    }
  };

  const handleSummarizeSelection = () => {
    if (selectedText) {
      setInputValue(`Can you summarize this: ${selectedText}`);
      setIsOpen(true);
      setShowContextMenu(false);
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
      }
    }
  };

  return (
    <div className={`chatbot-container ${isOpen ? 'chatbot-container--expanded' : 'chatbot-container--docked'}`} ref={chatContainerRef}>
      {/* Context menu for selected text */}
      {showContextMenu && (
        <div
          className="chatbot-context-menu"
          ref={contextMenuRef}
          style={{
            position: 'fixed',
            left: contextMenuPosition.x,
            top: contextMenuPosition.y,
            zIndex: 1001,
          }}
        >
          <div className="chatbot-context-menu-item" onClick={handleAskAboutSelection}>
            <span>🤖 Ask about this</span>
          </div>
          <div className="chatbot-context-menu-item" onClick={handleAskClarifyingQuestion}>
            <span>❓ Explain simply</span>
          </div>
          <div className="chatbot-context-menu-item" onClick={handleSummarizeSelection}>
            <span>📝 Summarize</span>
          </div>
        </div>
      )}

      {/* Docked state - always visible button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'chatbot-toggle--open' : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        <div className="chatbot-icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M12 2C6.48 2 2 6.48 2 12C2 13.54 2.36 15.01 3.02 16.32L2 22L7.68 20.98C8.99 21.64 10.46 22 12 22C17.52 22 22 17.52 22 12C22 6.48 17.52 2 12 2ZM12 20C8.69 20 6 17.31 6 14C6 10.69 8.69 8 12 8C15.31 8 18 10.69 18 14C18 17.31 15.31 20 12 20ZM13 9L10 12L13 15L13.7 14.3L11.4 12L13.7 9.7L13 9Z" fill="currentColor"/>
          </svg>
        </div>
        {selectedText && !showContextMenu && (
          <div className="chatbot-quick-ask" onClick={handleAskAboutSelection}>
            <span>Ask about selection</span>
          </div>
        )}
      </button>

      {/* Expanded chat interface */}
      {isOpen && (
        <div className="chatbot-panel">
          <div className="chatbot-header">
            <div className="chatbot-title">
              <span>AI Assistant</span>
            </div>
            <button
              className="chatbot-close"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M19 6.41L17.59 5L12 10.59L6.41 5L5 6.41L10.59 12L5 17.59L6.41 19L12 13.41L17.59 19L19 17.59L13.41 12L19 6.41Z" fill="currentColor"/>
              </svg>
            </button>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <h3>Hello! I'm your AI assistant.</h3>
                <p>Ask me anything about this book. You can also select text and click "Ask about selection" to get context-aware answers.</p>
              </div>
            ) : (
              messages.map((message) => (
                <ChatMessage
                  key={message.id}
                  text={message.text}
                  sender={message.sender}
                  timestamp={message.timestamp}
                  citations={message.citations}
                  sources={message.sources}
                />
              ))
            )}
            {isLoading && (
              <ChatMessage
                text=""
                sender="bot"
                isLoading={true}
              />
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                  e.preventDefault();
                  handleSendMessage();
                }
              }}
              placeholder="Ask a question about this book..."
              className="chatbot-input"
              rows={1}
              aria-label="Type your message"
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="chatbot-send-button"
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                <path d="M2.01 21L23 12L2.01 3L2 10L17 12L2 14L2.01 21Z" fill="currentColor"/>
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;