import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  icon: string;
  description: ReactNode;
  link?: string;
  highlight?: boolean;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Fundamentals',
    icon: '🤖',
    description: (
      <>
        Master Robot Operating System 2 with comprehensive tutorials on publishers,
        subscribers, services, and actions. Build real-world robotics applications
        from scratch.
      </>
    ),
    link: '/docs/module-1-ros2-fundamentals/chapter-1-publishers',
  },
  {
    title: 'NVIDIA Isaac Sim',
    icon: '🎮',
    description: (
      <>
        Learn simulation-driven development with NVIDIA Isaac Sim. Create realistic
        robot simulations, test algorithms, and accelerate development workflows.
      </>
    ),
    link: '/docs/module-2-isaac-sim/chapter-1-introduction',
  },
  {
    title: 'Humanoid Control',
    icon: '🦾',
    description: (
      <>
        Explore advanced topics in humanoid robotics including inverse kinematics,
        motion planning, balance control, and real-time decision making.
      </>
    ),
    link: '/docs/intro',
  },
  {
    title: 'Hands-On Projects',
    icon: '⚡',
    description: (
      <>
        Apply your knowledge through practical projects and exercises. Each chapter
        includes working code examples you can run and modify.
      </>
    ),
    highlight: true,
  },
  {
    title: 'Industry Standards',
    icon: '🏆',
    description: (
      <>
        Learn industry-standard tools and best practices used by professional robotics
        engineers at leading companies and research labs.
      </>
    ),
    highlight: true,
  },
  {
    title: 'Open Source',
    icon: '💡',
    description: (
      <>
        Completely free and open source. Contribute to the textbook, suggest improvements,
        and help build the robotics education community.
      </>
    ),
    highlight: true,
  },
];

function Feature({title, icon, description, link, highlight}: FeatureItem) {
  const content = (
    <>
      <div className={styles.featureIcon}>{icon}</div>
      <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
      <p className={styles.featureDescription}>{description}</p>
      {link && (
        <Link className={styles.featureLink} to={link}>
          Explore Module →
        </Link>
      )}
    </>
  );

  return (
    <div className={clsx('col col--4', styles.featureCol)}>
      <div className={clsx(styles.featureCard, {[styles.featureHighlight]: highlight})}>
        {content}
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featuresHeader}>
          <Heading as="h2" className={styles.featuresTitle}>
            Everything You Need to Master Robotics
          </Heading>
          <p className={styles.featuresSubtitle}>
            Comprehensive curriculum designed for students, researchers, and professionals
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
