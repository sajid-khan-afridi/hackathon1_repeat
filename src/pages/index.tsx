import type { ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import { FadeIn, CountUp } from '@site/src/components/animations';

import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className={styles.heroContent}>
        <div className={styles.heroText}>
          <FadeIn variant="fade-up" staggerIndex={0} triggerOnScroll={false}>
            <span className={styles.heroLabel}>Interactive Learning Platform</span>
          </FadeIn>
          <FadeIn variant="fade-up" staggerIndex={1} triggerOnScroll={false}>
            <Heading as="h1" className={styles.heroTitle}>
              Master Physical AI & Humanoid Robotics
            </Heading>
          </FadeIn>
          <FadeIn variant="fade-up" staggerIndex={2} triggerOnScroll={false}>
            <p className={styles.heroSubtitle}>
              Comprehensive textbook covering ROS 2, NVIDIA Isaac Sim, and advanced robotics concepts.
              From fundamentals to real-world applications.
            </p>
          </FadeIn>
          <FadeIn variant="fade-up" staggerIndex={3} triggerOnScroll={false}>
            <div className={styles.heroButtons}>
              <Link
                className={clsx('button button--primary button--lg', styles.primaryButton)}
                to="/docs/intro"
              >
                Start Learning
              </Link>
              <Link
                className={clsx('button button--outline button--lg', styles.secondaryButton)}
                to="/docs/module-1-ros2-fundamentals/chapter-1-publishers"
              >
                Explore Modules
              </Link>
            </div>
          </FadeIn>
          <FadeIn variant="fade-up" staggerIndex={4} triggerOnScroll={false}>
            <div className={styles.heroStats}>
              <div className={styles.stat}>
                <span className={styles.statNumber}>
                  <CountUp end={10} duration={1500} suffix="+" />
                </span>
                <span className={styles.statLabel}>Chapters</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>
                  <CountUp end={3} duration={1500} delay={200} />
                </span>
                <span className={styles.statLabel}>Core Modules</span>
              </div>
              <div className={styles.stat}>
                <span className={styles.statNumber}>
                  <CountUp end={100} duration={1500} delay={400} suffix="%" />
                </span>
                <span className={styles.statLabel}>Free & Open</span>
              </div>
            </div>
          </FadeIn>
        </div>
        <FadeIn variant="fade-left" staggerIndex={2} duration={600} triggerOnScroll={false}>
          <div className={styles.heroVisual}>
            <div className={styles.visualCard}>
              <div className={styles.codePreview}>
                <div className={styles.codeHeader}>
                  <span className={styles.codeDot}></span>
                  <span className={styles.codeDot}></span>
                  <span className={styles.codeDot}></span>
                  <span className={styles.codeTitle}>ros2_publisher.py</span>
                </div>
                <pre className={styles.codeContent}>{`import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('publisher')
    pub = node.create_publisher(
        String, 'topic', 10
    )
    # Start publishing...`}</pre>
              </div>
            </div>
          </div>
        </FadeIn>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics with comprehensive tutorials on ROS 2, NVIDIA Isaac Sim, and advanced robotics concepts."
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
