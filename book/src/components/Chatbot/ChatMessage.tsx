import React from 'react';

interface Source {
  title: string;
  url?: string;
  content: string;
}

interface ChatMessageProps {
  text: string;
  sender: 'user' | 'bot';
  timestamp?: Date;
  citations?: string[];
  sources?: Source[];
  isLoading?: boolean;
}

const ChatMessage: React.FC<ChatMessageProps> = ({
  text,
  sender,
  timestamp,
  citations,
  sources,
  isLoading = false
}) => {
  const formatTime = (date?: Date) => {
    if (!date) return '';
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={`chat-message chat-message--${sender}`}>
      <div className="chat-message-content">
        {isLoading ? (
          <div className="chat-typing-indicator">
            <div className="chat-typing-dot"></div>
            <div className="chat-typing-dot"></div>
            <div className="chat-typing-dot"></div>
          </div>
        ) : text ? (
          text
        ) : (
          <div className="chat-message-error">
            Failed to load message content
          </div>
        )}
      </div>
      {citations && citations.length > 0 && (
        <div className="chat-citations">
          <div className="chat-citations-label">Sources:</div>
          {citations.map((citation, index) => (
            <span key={index} className="chat-citation">
              [{index + 1}] {citation}
            </span>
          ))}
        </div>
      )}
      {sources && sources.length > 0 && (
        <div className="chat-sources">
          <details className="chat-sources-details">
            <summary>Sources referenced</summary>
            {sources.map((source, index) => (
              <div key={index} className="chat-source-item">
                <div className="chat-source-title">
                  {source.url ? (
                    <a href={source.url} target="_blank" rel="noopener noreferrer">
                      {source.title}
                    </a>
                  ) : (
                    source.title
                  )}
                </div>
                <div className="chat-source-content">
                  {source.content.substring(0, 100)}...
                </div>
              </div>
            ))}
          </details>
        </div>
      )}
      {timestamp && !isLoading && (
        <div className="chat-message-timestamp">
          {formatTime(timestamp)}
        </div>
      )}
    </div>
  );
};

export default ChatMessage;