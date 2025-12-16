/**
 * Chatbot Page - Interactive RAG Q&A Interface
 *
 * Dedicated page for the AI chatbot that answers questions
 * about the robotics textbook content using RAG (Retrieval-Augmented Generation).
 */

import React from 'react';
import Layout from '@theme/Layout';
import ChatbotWidget from '@site/src/components/ChatbotWidget';
import styles from './chatbot.module.css';

export default function ChatbotPage(): JSX.Element {
  return (
    <Layout
      title="AI Chatbot"
      description="Ask questions about robotics, ROS 2, and NVIDIA Isaac Sim"
    >
      <div className={styles.chatbotPage}>
        <div className={styles.chatbotContainer}>
          <header className={styles.chatbotPageHeader}>
            <h1>Ask the Robotics AI Assistant</h1>
            <p className={styles.chatbotDescription}>
              Get instant answers about ROS 2, NVIDIA Isaac Sim, and robotics concepts
              from our AI-powered assistant trained on the complete textbook.
            </p>
          </header>

          <div className={styles.chatbotWidgetWrapper}>
            <ChatbotWidget />
          </div>

          <footer className={styles.chatbotFooter}>
            <div className={styles.features}>
              <div className={styles.feature}>
                <h3>🎯 Context-Aware</h3>
                <p>Answers based on textbook content with source citations</p>
              </div>
              <div className={styles.feature}>
                <h3>💬 Conversational</h3>
                <p>Maintains conversation history for follow-up questions</p>
              </div>
              <div className={styles.feature}>
                <h3>📚 Comprehensive</h3>
                <p>Covers all modules from ROS 2 basics to advanced topics</p>
              </div>
            </div>

            <div className={styles.tips}>
              <h3>💡 Tips for Better Answers:</h3>
              <ul>
                <li>Ask specific questions (e.g., "How do I create a ROS 2 publisher?")</li>
                <li>Mention the module or topic if you know it</li>
                <li>Use follow-up questions to dive deeper</li>
                <li>Check the source citations for more details</li>
              </ul>
            </div>
          </footer>
        </div>
      </div>
    </Layout>
  );
}
