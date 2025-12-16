import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div style={{
          display: 'flex',
          flexDirection: 'row',
          alignItems: 'center',
          gap: '3rem',
          flexWrap: 'wrap'
        }}>
          <div style={{ flex: '1 1 400px', minWidth: '300px' }}>
            <Heading as="h1" className="hero__title">
              Physical AI & Humanoid Robotics
            </Heading>
            <p className="hero__subtitle">
              Building Intelligent Embodied Systems with ROS 2, Digital Twins, and Vision-Language-Action Models
            </p>
            <p style={{ fontSize: '1.1rem', marginTop: '1.5rem', marginBottom: '2rem' }}>
              A comprehensive technical textbook for students and practitioners learning to build intelligent physical AI systems,
              from robot operating systems to vision-language-action models for humanoid robotics.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                📘 Start Reading the Book
              </Link>
            </div>
          </div>
          <div style={{ flex: '1 1 400px', minWidth: '300px', textAlign: 'center' }}>
            <img
              src="/img/physical-ai-hero.png"
              alt="Physical AI and Humanoid Robotics Illustration"
              style={{
                width: '100%',
                height: 'auto',
                borderRadius: '8px',
                boxShadow: '0 4px 12px rgba(0, 0, 0, 0.1)'
              }}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A comprehensive textbook on building intelligent embodied systems with ROS 2, Digital Twins, and Vision-Language-Action Models">
      <HomepageHeader />
      <main style={{ minHeight: '50vh' }}>
      </main>
    </Layout>
  );
}
