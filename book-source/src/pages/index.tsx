import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/Physical-AI-Humanoid-Robotics/">
              Start Learning →
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/Physical-AI-Humanoid-Robotics/ros2-nervous-system/">
              Explore Modules
            </Link>
          </div>

          {/* Stats Bar */}
          <div className={styles.statsBar}>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>4</div>
              <div className={styles.statLabel}>Modules</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>16+</div>
              <div className={styles.statLabel}>Lessons</div>
            </div>
            <div className={styles.statItem}>
              <div className={styles.statNumber}>AI</div>
              <div className={styles.statLabel}>Powered</div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

type ModuleItem = {
  number: string;
  title: string;
  description: string;
  link: string;
  icon: string;
};

const ModuleList: ModuleItem[] = [
  {
    number: '01',
    title: 'The Robotic Nervous System',
    description: 'Master ROS 2 as the communication backbone for humanoid robots. Learn node communication, Python integration with rclpy, and robot description with URDF.',
    link: '/docs/Physical-AI-Humanoid-Robotics/ros2-nervous-system/',
    icon: '🧠',
  },
  {
    number: '02',
    title: 'Sensors & Perception',
    description: 'Discover how humanoid robots sense their environment through cameras, depth sensors, IMUs, and advanced sensor fusion techniques for spatial awareness.',
    link: '/docs/Physical-AI-Humanoid-Robotics/sensors-perception/',
    icon: '👁️',
  },
  {
    number: '03',
    title: 'The AI-Robot Brain',
    description: 'Explore NVIDIA Isaac platform for photorealistic simulation, hardware-accelerated perception, and specialized navigation for bipedal robots.',
    link: '/docs/Physical-AI-Humanoid-Robotics/isaac-ai-brain/',
    icon: '🤖',
  },
  {
    number: '04',
    title: 'Vision-Language-Action',
    description: 'Build end-to-end autonomous behavior using foundation models, vision-language integration, and cognitive planning for humanoid intelligence.',
    link: '/docs/Physical-AI-Humanoid-Robotics/vision-language-action/',
    icon: '🎯',
  },
];

function Module({number, title, description, link, icon}: ModuleItem) {
  return (
    <div className={clsx('col col--6', styles.moduleCard)}>
      <div className="card">
        <div className={styles.moduleNumber}>{number}</div>
        <div className="card__header">
          <h3>{icon} {title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--primary button--block" to={link}>
            Explore Module →
          </Link>
        </div>
      </div>
    </div>
  );
}

function HomepageModules() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Curriculum</span>
          <h2 className={styles.sectionTitle}>Master Humanoid Robotics</h2>
          <p className={styles.sectionSubtitle}>
            A comprehensive journey from foundational robotic systems to cutting-edge AI integration.
            Each module builds upon the last to create complete mastery.
          </p>
        </div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

type FeatureItem = {
  icon: string;
  title: string;
  description: string;
};

const FeatureList: FeatureItem[] = [
  {
    icon: '🤖',
    title: 'AI-Powered Assistant',
    description: 'Get instant answers with our built-in AI chatbot. Ask questions about any topic and receive contextual explanations from the course materials.',
  },
  {
    icon: '🎓',
    title: 'Expert-Crafted Content',
    description: 'Learn from industry best practices, common pitfalls, and production-ready approaches used by experienced robotics engineers.',
  },
  {
    icon: '⚡',
    title: 'Hands-On Projects',
    description: 'Apply concepts through real-world design challenges, capstone projects, and integration exercises with actual robotics frameworks.',
  },
];

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Why Choose Us</span>
          <h2 className={styles.sectionTitle}>Learn Smarter, Build Faster</h2>
          <p className={styles.sectionSubtitle}>
            Everything you need to become a proficient humanoid robotics engineer.
          </p>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <div key={idx} className="col col--4">
              <div className={styles.featureCard}>
                <span className={styles.featureIcon}>{props.icon}</span>
                <h3>{props.title}</h3>
                <p>{props.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageCTA() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
        <p className={styles.ctaSubtitle}>
          Start your journey into Physical AI and humanoid robotics today.
        </p>
        <Link
          className="button button--primary button--lg"
          to="/docs/Physical-AI-Humanoid-Robotics/">
          Get Started Free →
        </Link>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="Learn the complete technology stack for building intelligent humanoid robots with Physical AI">
      <HomepageHeader />
      <main>
        <HomepageModules />
        <HomepageFeatures />
        <HomepageCTA />
      </main>
    </Layout>
  );
}
