import React, { useState, useEffect } from 'react';
import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';
import { useAuth } from '../hooks/useAuth';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  const baseUrl = siteConfig.baseUrl || '/';
  const { isAuthenticated, isLoading, user, profile } = useAuth();

  // Fallback: Use isLoggedIn flag if Better-Auth session is failing
  const isLoggedInFlag = typeof window !== 'undefined' ? localStorage.getItem('isLoggedIn') === 'true' : false;
  const isLoggedIn = isAuthenticated || isLoggedInFlag;

  // 🐛 DEBUG: Log session state on mount and when it changes
  useEffect(() => {
    const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;
    const storedUser = typeof window !== 'undefined' ? localStorage.getItem('user') : null;
    const storedProfile = typeof window !== 'undefined' ? localStorage.getItem('profile') : null;

    console.log('[Homepage] Current Session:', {
      isAuthenticated,
      isLoading,
      user: user ? { id: user.id, email: user.email } : null,
      profile: profile ? {
        software_background: profile.software_background,
        hardware_experience: profile.hardware_experience
      } : null,
      localStorage: {
        hasToken: !!token,
        hasUser: !!storedUser,
        hasProfile: !!storedProfile
      }
    });
  }, [isAuthenticated, isLoading, user, profile]);

  // Protected feature handler
  const handleProtectedFeature = (e: React.MouseEvent, featureName: string, storageKey: string, storageValue: string) => {
    // Check Better Auth session + fallback
    const token = typeof window !== 'undefined' ? localStorage.getItem('auth_token') : null;
    const storedUser = typeof window !== 'undefined' ? localStorage.getItem('user') : null;
    const isLoggedInFlag = typeof window !== 'undefined' ? localStorage.getItem('isLoggedIn') : null;

    // Fallback: Use isLoggedIn flag if Better-Auth session is failing
    const isLoggedIn = isAuthenticated || isLoggedInFlag === 'true';

    console.log('[Homepage] Feature clicked - Auth state:', {
      featureName,
      isAuthenticated,
      isLoggedIn,
      isLoading,
      hasToken: !!token,
      hasUser: !!storedUser,
      hasIsLoggedInFlag: isLoggedInFlag === 'true',
      user,
      profile
    });

    // Wait for auth loading to complete - DON'T assume logged out during loading
    if (isLoading) {
      e.preventDefault();
      console.log('[Homepage] Still loading auth state, please wait...');
      return false;
    }

    if (!isLoggedIn || !token || !storedUser) {
      e.preventDefault();

      // Store target page for redirect after login
      if (typeof window !== 'undefined') {
        // Store the feature tag destination as return URL
        const targetUrl = (e.currentTarget as HTMLAnchorElement).getAttribute('href') || '/physical_ai_textbook/docs/intro';
        localStorage.setItem('auth_redirect_url', targetUrl);
        console.log('[Homepage] Not authenticated, redirecting to login. Return URL:', targetUrl);
      }

      alert(`Bulldog's AI features are exclusive for members. Please Login to continue.`);
      setTimeout(() => {
        window.location.href = window.location.origin + '/physical_ai_textbook/login';
      }, 500);
      return false;
    }
    // If authenticated, allow the feature
    if (typeof window !== 'undefined') {
      localStorage.setItem(storageKey, storageValue);
    }
    return true;
  };

  return (
    <header className={styles.heroSection}>
      {/* Animated gradient background */}
      <div className={styles.gradientBackground}></div>

      {/* Floating particles effect */}
      <div className={styles.particlesContainer}>
        {[...Array(20)].map((_, i) => (
          <div key={i} className={styles.particle} style={{
            left: `${Math.random() * 100}%`,
            animationDelay: `${Math.random() * 5}s`,
            animationDuration: `${5 + Math.random() * 10}s`
          }}></div>
        ))}
      </div>

      <div className={styles.heroContent}>
        {/* Main headline */}
        <h1 className={styles.heroTitle}>
          <span className={styles.titleGradient}>Physical AI</span>
          <br />
          <span className={styles.titleAccent}>& Humanoid Robotics</span>
        </h1>

        {/* Subtitle */}
        <p className={styles.heroSubtitle}>
          Build Intelligent Embodied Systems with ROS 2, Digital Twins,
          <br />
          and Vision-Language-Action Models
        </p>

        {/* Feature tags - All PUBLIC - AI features are protected at the button level on chapter pages */}
        <div className={styles.featureTags}>
          <Link
            to={`${baseUrl}docs/intro`}
            className={`${styles.tag} button button--primary`}
            onClick={() => {
              // PUBLIC: Allow all guests to access intro page
              // Set flag for Bulldog auto-open if logged in
              if (isLoggedIn && typeof window !== 'undefined') {
                localStorage.setItem('auto_open_bulldog', 'true');
                localStorage.setItem('from_ai_powered', 'true');
              }
            }}
          >
            🤖 AI-Powered Learning
          </Link>
          <Link
            to={`${baseUrl}docs/table-of-contents`}
            className={`${styles.tag} button button--primary`}
            onClick={() => {
              // PUBLIC: Allow all guests to view table of contents
              if (isLoggedIn && typeof window !== 'undefined') {
                localStorage.setItem('trigger_urdu_toc', 'true');
              }
            }}
          >
            🌍 Multilingual (Urdu)
          </Link>
          <Link
            to={`${baseUrl}docs/table-of-contents`}
            className={`${styles.tag} button button--primary`}
            onClick={() => {
              // PUBLIC: Allow all guests to view table of contents
              if (isLoggedIn && typeof window !== 'undefined') {
                localStorage.setItem('show_personalized', 'true');
              }
            }}
          >
            ✨ Personalized Content
          </Link>
        </div>

        {/* CTA Buttons - Using baseUrl for correct routing */}
        <div className={styles.ctaButtons}>
          <Link className={styles.primaryButton} to={`${baseUrl}docs/intro`}>
            <span className={styles.buttonGlow}></span>
            <span className={styles.buttonText}>🚀 Get Started</span>
          </Link>

          <Link className={styles.secondaryButton} to={`${baseUrl}login`}>
            <span className={styles.buttonText}>📝 Login</span>
          </Link>
        </div>

        {/* Stats bar - Interactive with links */}
        <div className={styles.statsBar}>
          <Link to={`${baseUrl}docs/table-of-contents`} className={styles.statLink} title="View all 16 chapters">
            <div className={styles.stat}>
              <div className={styles.statNumber}>16</div>
              <div className={styles.statLabel}>Chapters</div>
            </div>
          </Link>
          <Link to={`${baseUrl}docs/table-of-contents`} className={styles.statLink} title="Explore 4 learning modules">
            <div className={styles.stat}>
              <div className={styles.statNumber}>4</div>
              <div className={styles.statLabel}>Modules</div>
            </div>
          </Link>
          <Link to={`${baseUrl}docs/table-of-contents`} className={styles.statLink} title="85,000+ words of content">
            <div className={styles.stat}>
              <div className={styles.statNumber}>85k+</div>
              <div className={styles.statLabel}>Words</div>
            </div>
          </Link>
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
      description="A comprehensive textbook on building intelligent embodied systems with ROS 2, Digital Twins, and Vision-Language-Action Models"
    >
      <HomepageHeader />
    </Layout>
  );
}
