import React, { useState, useEffect } from 'react';
import styles from './Chatbot.module.css';

interface ChatInputProps {
  onSendMessage: (message: string, selectedText?: string) => void;
  isLoading: boolean;
  selectedText: string;
  onClearSelectedText: () => void;
}

const ChatInput: React.FC<ChatInputProps> = ({
  onSendMessage,
  isLoading,
  selectedText,
  onClearSelectedText
}) => {
  const [inputValue, setInputValue] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (inputValue.trim() && !isLoading) {
      onSendMessage(inputValue, selectedText);
      setInputValue('');
      if (selectedText) {
        onClearSelectedText();
      }
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.chatInputContainer}>
      {selectedText && (
        <div style={{ fontSize: '12px', marginBottom: '5px', color: '#666' }}>
          Using selected text: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
          <button
            type="button"
            onClick={onClearSelectedText}
            style={{ marginLeft: '8px', fontSize: '10px', color: '#999', background: 'none', border: 'none', cursor: 'pointer' }}
          >
            Clear
          </button>
        </div>
      )}
      <textarea
        className={styles.chatInput}
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        placeholder="Ask a question about the book..."
        rows={1}
        disabled={isLoading}
      />
      <button
        type="submit"
        className={styles.sendButton}
        disabled={isLoading || !inputValue.trim()}
        aria-label="Send message"
      >
        ➤
      </button>
    </form>
  );
};

export default ChatInput;