---
id: 3
title: Enhanced Homepage with Animations and Floating Robots
stage: general
date_iso: 2025-12-07
surface: agent
model: gemini
feature: none
branch: main
user: laiba noman
command:
  - write_file:
      file_path: src/css/custom.css
      content: |
        /**
         * Any CSS included here will be global. The classic template
         * bundles Infima by default. Infima is a CSS framework designed to
         * work well for content-centric websites.
         */

        /* You can override the default Infima variables here. */
        :root {
          --ifm-color-primary: #4169E1; /* Royal Blue */
          --ifm-color-primary-dark: #3a5ed0;
          --ifm-color-primary-darker: #3657c4;
          --ifm-color-primary-darkest: #2d47a2;
          --ifm-color-primary-light: #4874f2;
          --ifm-color-primary-lighter: #4d7af6;
          --ifm-color-primary-lightest: #6a95ff;
          --ifm-color-secondary: #000000; /* Black */
          --ifm-background-color: #f0f2f5; /* Light grey for light mode background */
          --ifm-font-family-base: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
          --ifm-font-color-base: #333333; /* Dark grey for text in light mode */
          --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
        }

        /* Dark mode styles */
        [data-theme='dark'] {
          --ifm-color-primary: #4169E1; /* Royal Blue */
          --ifm-color-primary-dark: #3a5ed0;
          --ifm-color-primary-darker: #3657c4;
          --ifm-color-primary-darkest: #2d47a2;
          --ifm-color-primary-light: #4874f2;
          --ifm-color-primary-lighter: #4d7af6;
          --ifm-color-primary-lightest: #6a95ff;
          --ifm-color-secondary: #ffffff; /* White for secondary elements in dark mode */
          --ifm-background-color: #121212; /* Dark background */
          --ifm-font-color-base: #ffffff; /* White for text in dark mode */
          --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.3);
        }

        .hero {
          background-color: var(--ifm-background-color);
          color: var(--ifm-font-color-base);
        }

        .hero__title {
          color: var(--ifm-color-primary);
        }

        .hero__subtitle {
          color: var(--ifm-font-color-base);
        }

        /* Custom styles for professional look */
        .navbar {
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        }

        [data-theme='dark'] .navbar {
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.4);
        }

        .footer {
          background-color: var(--ifm-color-secondary);
          color: var(--ifm-font-color-base);
          padding: 2rem;
          text-align: center;
        }

        [data-theme='dark'] .footer {
          background-color: #000000;
          color: #ffffff;
        }

        .featureSvg {
          max-width: 200px; /* Adjust size of feature images */
          height: auto;
          margin-bottom: 1rem;
          border-radius: 8px; /* Slightly rounded corners for images */
        }

        /* Homepage specific styles */
        .homepage-section {
          padding: 4rem 0;
          text-align: center;
        }

        .homepage-section:nth-child(even) {
          background-color: var(--ifm-background-color);
        }

        .homepage-section:nth-child(odd) {
          background-color: var(--ifm-color-secondary);
          color: var(--ifm-font-color-base);
        }

        [data-theme='dark'] .homepage-section:nth-child(odd) {
          background-color: #0d0d0d; /* Slightly lighter dark for contrast */
        }

        .section-title {
          font-size: 2.5rem;
          margin-bottom: 2rem;
          color: var(--ifm-color-primary);
        }

        .section-description {
          font-size: 1.1rem;
          max-width: 800px;
          margin: 0 auto 3rem auto;
          line-height: 1.6;
        }

        .book-cover {
          max-width: 300px;
          border-radius: 8px;
          box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
        }

        .review-card {
          background-color: var(--ifm-background-color);
          border-radius: 8px;
          padding: 1.5rem;
          margin-bottom: 1.5rem;
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.05);
          text-align: left;
        }

        [data-theme='dark'] .review-card {
          background-color: #222222;
          box-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
        }

        .review-text {
          font-style: italic;
          margin-bottom: 1rem;
        }

        .review-author {
          font-weight: bold;
          color: var(--ifm-color-primary);
        }

        /* Floating animation for robots */
        @keyframes float {
          0% { transform: translateY(0px); }
          50% { transform: translateY(-10px); }
          100% { transform: translateY(0px); }
        }

        .float-animation {
          animation: float 3s ease-in-out infinite;
        }

        /* Generic arrow pointing effect */
        .pointing-arrow {
          position: relative;
          display: inline-block; /* Or block, depending on layout */
        }

        .pointing-arrow::after {
          content: '→'; /* Unicode right arrow */
          position: absolute;
          font-size: 2rem;
          color: var(--ifm-color-primary);
          /* Adjust positioning as needed */
          top: 50%;
          right: -30px; /* Distance from the element */
          transform: translateY(-50%);
        }
  - write_file:
      file_path: src/pages/index.tsx
      content: |
        import type {ReactNode} from 'react';
        import clsx from 'clsx';
        import Link from '@docusaurus/Link';
        import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
        import Layout from '@theme/Layout';
        import HomepageFeatures from '@site/src/components/HomepageFeatures';
        import Heading from '@theme/Heading';

        import styles from './index.module.css';

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
                    <img src="https://via.placeholder.com/400x500/4169E1/FFFFFF?text=Book+Cover" alt="Book Cover" className="book-cover" />
                  </div>
                  <div className="col col--6 text--left">
                    <Heading as="h2" className="section-title">About the Book</Heading>
                    <p className="section-description">
                      "Physical AI & Humanoid Robotics" delves into the intricate world where artificial intelligence meets robotic hardware. Discover cutting-edge research, practical applications, and the ethical considerations of creating intelligent machines that interact with our physical world. From perception and control to learning and adaptation, this book is your comprehensive guide to understanding and building the next generation of humanoid robots.
                    </p>
                    <p className="section-description">
                      Ideal for students, researchers, and engineers, it covers topics suchs as advanced kinematics, dynamic control, reinforcement learning for robotics, human-robot interaction, and the future of sentient AI.
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
                        src="https://via.placeholder.com/150/4169E1/FFFFFF?text=Robot+1"
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
                        src="https://via.placeholder.com/150/4169E1/FFFFFF?text=Robot+2"
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
                        src="https://via.placeholder.com/150/4169E1/FFFFFF?text=Robot+3"
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
            </Layout>
          );
        }
