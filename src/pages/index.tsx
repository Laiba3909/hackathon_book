import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import ChatbotPopup from '@site/src/components/ChatbotPopup'; // Import the ChatbotPopup component

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Explore the Book
          </Link>
        </div>
      </div>
    </header>
  );
}

function AboutSection() {
  return (
    <section className="homepage-section">
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <img src="/img/robotb.png" alt="Book Cover" className="book-cover" />
          </div>
          <div className="col col--6 text--left">
            <Heading as="h2" className="section-title">About the Book</Heading>
            <p className="section-description">
              "Physical AI & Humanoid Robotics" delves into the intricate world where artificial intelligence meets robotic hardware. Discover cutting-edge research, practical applications, and the ethical considerations of creating intelligent machines that interact with our physical world. From perception and control to learning and adaptation, this book is your comprehensive guide to understanding and building the next generation of humanoid robots.
            </p>
            <p className="section-description">
              Ideal for students, researchers, and engineers, it covers topics such as advanced kinematics, dynamic control, reinforcement learning for robotics, human-robot interaction, and the future of sentient AI.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function AdditionalContentSection() {
  return (
    <section className="homepage-section">
      <div className="container">
        <Heading as="h2" className="section-title">Discover More Chapters</Heading>
        <div className="row" style={{ alignItems: 'center' }}>
          <div className="col col--4">
            <div className="pointing-arrow" style={{ textAlign: 'right' }}>
              <img
                src="/img/robot1.png"
                alt="Robot 1"
                className="float-animation"
                style={{ maxWidth: '150px' }}
              />
            </div>
          </div>
          <div className="col col--8 text--left">
            <Heading as="h3">Chapter 1: Foundations of Physical AI</Heading>
            <p>
              Explore the fundamental principles that govern intelligent agents interacting with the physical world.
              Learn about sensor fusion, inverse kinematics, and real-time control systems crucial for robotic movement.
            </p>
          </div>
        </div>

        <div className="row" style={{ alignItems: 'center', marginTop: '3rem' }}>
          <div className="col col--8 text--right">
            <Heading as="h3">Chapter 2: Humanoid Design & Control</Heading>
            <p>
              Dive deep into the architecture of humanoid robots, from their mechanical design to advanced balancing and locomotion algorithms.
              Understand the challenges and innovations in bipedalism.
            </p>
          </div>
          <div className="col col--4">
            <div className="pointing-arrow" style={{ textAlign: 'left', transform: 'scaleX(-1)' }}> {/* Flip arrow for left pointing */}
              <img
                src="/img/robot2.png"
                alt="Robot 2"
                className="float-animation"
                style={{ maxWidth: '150px' }}
              />
            </div>
          </div>
        </div>

        <div className="row" style={{ alignItems: 'center', marginTop: '3rem' }}>
          <div className="col col--4">
            <div className="pointing-arrow" style={{ textAlign: 'right' }}>
              <img
                src="/img/robot3.png"
                alt="Robot 3"
                className="float-animation"
                style={{ maxWidth: '150px' }}
              />
            </div>
          </div>
          <div className="col col--8 text--left">
            <Heading as="h3">Chapter 3: AI-Driven Perception</Heading>
            <p>
              Unravel the secrets of how robots see and understand their environment. From computer vision to LiDAR and depth sensing,
              discover how AI algorithms process sensory data for navigation and object manipulation.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}


function ReviewsSection() {
  const reviews = [
    {
      id: 1,
      text: "An indispensable guide for anyone serious about robotics. The depth of coverage is simply astounding!",
      author: "Dr. Evelyn Reed, Robotics Professor"
    },
    {
      id: 2,
      text: "Finally, a book that bridges the gap between theoretical AI and its physical embodiment. A must-read!",
      author: "Prof. Alan Turing, AI Researcher"
    },
    {
      id: 3,
      text: "The practical examples and clear explanations make complex topics accessible. Highly recommended!",
      author: "Sarah Connor, Robotics Engineer"
    }
  ];

  return (
    <section className="homepage-section">
      <div className="container">
        <Heading as="h2" className="section-title">What Readers Say</Heading>
        <div className="row">
          {reviews.map(review => (
            <div key={review.id} className="col col--4">
              <div className="review-card">
                <p className="review-text">"{review.text}"</p>
                <p className="review-author">- {review.author}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function GetStartedSection() {
  return (
    <section className="homepage-section">
      <div className="container">
        <Heading as="h2" className="section-title">Ready to Dive In?</Heading>
        <p className="section-description">
          Begin your journey into Physical AI & Humanoid Robotics today. Explore the comprehensive documentation or get your copy of the book!
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Reading Now
          </Link>
          <Link
            className="button button--outline button--lg"
            to="#"> {/* Placeholder for purchase link */}
            Purchase the Book
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Explore the world of Physical AI and Humanoid Robotics with this comprehensive book.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <AboutSection />
        <AdditionalContentSection /> {/* New section added here */}
        <ReviewsSection />
        <GetStartedSection />
      </main>
      <ChatbotPopup /> {/* Add the ChatbotPopup component here */}
    </Layout>
  );
}
