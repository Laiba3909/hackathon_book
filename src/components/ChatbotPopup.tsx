// src/components/ChatbotPopup.tsx
import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import FlexSearch from 'flexsearch';
import styles from './ChatbotPopup.module.css';

// Define the structure of a message in the chat
interface Message {
  text: string;
  sender: 'user' | 'bot';
}

// Define the structure of a document in our search index
interface SearchDoc {
    id: number;
    title: string;
    content: string;
    filepath: string;
}

const ChatbotPopup: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  
  // Refs for the search index and the message container
  const index = useRef(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // --- Index Loading Logic ---
  useEffect(() => {
    // This effect runs once when the component mounts
    setIsLoading(true);
    console.log("Initializing chatbot and loading search index...");

    // Create a new FlexSearch Document index instance
    index.current = new FlexSearch.Document({
        document: {
          id: 'id',
          index: ['title', 'content'],
          store: ['title', 'content', 'filepath'], // Store fields to show in results
        },
        tokenize: 'forward',
    });

    // Fetch the pre-built index from the static directory
    fetch('/search-index.json')
      .then(response => response.json())
      .then(exportedIndex => {
        // Import each part of the exported index
        for (const key in exportedIndex) {
          index.current.import(key, exportedIndex[key]);
        }
        console.log("Chatbot index loaded successfully.");
        setIsLoading(false);
        // Add a welcome message
        setMessages([{ text: "Hello! I am a standalone chatbot for this book. Ask me anything about the content.", sender: 'bot' }]);
      })
      .catch(error => {
        console.error('Error loading search index:', error);
        setIsLoading(false);
        setMessages([{ text: "I'm sorry, my knowledge base failed to load. I am unable to answer questions right now.", sender: 'bot' }]);
      });
  }, []); // The empty dependency array ensures this runs only once

  // --- UI and Message Handling Logic ---
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };
  useEffect(scrollToBottom, [messages]);

  const toggleChatbot = () => setIsOpen(!isOpen);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInputText(e.target.value);
  };

  // This function is called when the user sends a message
  const sendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (inputText.trim() === '' || isLoading) return;

    const userMessage: Message = { text: inputText, sender: 'user' };
    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    // Simulate a slight delay for a better user experience
    setTimeout(() => {
      // Perform the search on the local index
      const searchResults = index.current.search(userMessage.text, {
        limit: 1, // We only need the top result
        enrich: true, // This provides the document from the store
      });

      let botResponse: Message;

      if (searchResults.length > 0 && searchResults[0].result.length > 0) {
        // We found a relevant section in the book
        const topResult = searchResults[0].result[0].doc as SearchDoc;
        const answerText = `Based on the section "${topResult.title}", here is the most relevant information I found:\n\n"${topResult.content}"\n\n(Source: ${topResult.filepath})`;
        botResponse = { text: answerText, sender: 'bot' };
      } else {
        // If no relevant results are found, use the required off-topic response
        botResponse = { text: "I can only assist with questions related to this book and its chatbot. I am unable to help with other topics.", sender: 'bot' };
      }

      setMessages(prev => [...prev, botResponse]);
      setIsLoading(false);
    }, 500); // 500ms delay
  };
  
  // --- JSX Rendering ---
  return (
    <>
      <button className={styles.chatButton} onClick={toggleChatbot} title="Ask a question">
        💬
      </button>

      {isOpen && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatbotHeader}>
            <span>Book Chatbot</span>
            <button className={styles.closeButton} onClick={toggleChatbot}>&times;</button>
          </div>
          <div className={styles.chatbotMessages}>
            {messages.map((msg, idx) => (
              <div key={idx} className={clsx(styles.message, styles[msg.sender])}>
                {/* Use pre-wrap to respect newlines in the bot's response */}
                <span style={{ whiteSpace: 'pre-wrap' }}>{msg.text}</span>
              </div>
            ))}
            {isLoading && <div className={clsx(styles.message, styles.bot)}><span>Typing...</span></div>}
            <div ref={messagesEndRef} />
          </div>
          <form className={styles.chatbotInput} onSubmit={sendMessage}>
            <input
              type="text"
              value={inputText}
              onChange={handleInputChange}
              placeholder={isLoading || !index.current ? "Loading knowledge..." : "Ask about the book..."}
              disabled={isLoading || !index.current}
            />
            <button type="submit" disabled={isLoading || !index.current}>Send</button>
          </form>
        </div>
      )}
    </>
  );
};

export default ChatbotPopup;