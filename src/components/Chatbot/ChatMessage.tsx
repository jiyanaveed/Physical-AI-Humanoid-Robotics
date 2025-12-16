import React from 'react';
import { SourceChunk } from './types';
import styles from './Chatbot.module.css';

interface ChatMessageProps {
  text: string;
  isUser: boolean;
  sources?: SourceChunk[];
}

const ChatMessage: React.FC<ChatMessageProps> = ({ text, isUser, sources }) => {
  return (
    <div className={`${styles.message} ${isUser ? styles.userMessage : styles.botMessage}`}>
      <p className={styles.messageText}>{text}</p>

      {sources && sources.length > 0 && !isUser && (
        <div className={styles.sources}>
          <strong>Sources:</strong>
          {sources.map((source, index) => (
            <div key={index} className={styles.sourceItem}>
              <a
                href={source.url}
                className={styles.sourceLink}
                target="_blank"
                rel="noopener noreferrer"
              >
                {source.page_title} ({source.section})
              </a>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default ChatMessage;