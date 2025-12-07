// src/components/ChatbotPopup.tsx
import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './ChatbotPopup.module.css'; // We'll create this CSS module

interface Message {
  text: string;
  sender: 'user' | 'bot';
}

const ChatbotPopup: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const API_BASE_URL = 'http://localhost:8000'; // FastAPI backend URL

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(scrollToBottom, [messages]);

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInputText(e.target.value);
  };

  const sendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (inputText.trim() === '') return;

    const userMessage: Message = { text: inputText, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ query: userMessage.text }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessage: Message = { text: data.response || 'Sorry, I could not process that.', sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Error sending message to backend:', error);
      const errorMessage: Message = { text: 'Oops! Something went wrong. Please try again.', sender: 'bot' };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      <button className={styles.chatButton} onClick={toggleChatbot} title="How can I help you?">
        {/* You can use an icon here, e.g., a chat bubble icon */}
        💬
      </button>

      {isOpen && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatbotHeader}>
            <span>How can I help you?</span>
            <button className={styles.closeButton} onClick={toggleChatbot}>
              &times;
            </button>
          </div>
          <div className={styles.chatbotMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={clsx(styles.message, styles[msg.sender])}>
                {msg.text}
              </div>
            ))}
            {isLoading && (
              <div className={clsx(styles.message, styles.bot)}>
                Typing...
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form className={styles.chatbotInput} onSubmit={sendMessage}>
            <input
              type="text"
              value={inputText}
              onChange={handleInputChange}
              placeholder="Ask me anything about the book..."
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
};

export default ChatbotPopup;
