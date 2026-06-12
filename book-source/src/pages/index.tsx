/**
 * Landing page — Physical AI & Humanoid Robotics Textbook
 * Portfolio-grade hero with animated conversation demo
 */

import type {ReactNode} from 'react';
import { useEffect, useState, useRef, useCallback } from 'react';
import { motion, useReducedMotion } from 'motion/react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import BrowserOnly from '@docusaurus/BrowserOnly';

import ConversationDemo from '../components/ConversationDemo/ConversationDemo';
import styles from './index.module.css';

/* ─── Detect if chat can be opened (browser-only) ─── */
const isBrowser = typeof window !== 'undefined';

/* ─── Staggered headline animation ─── */
const headlineWords = ['Physical AI', '&', 'Humanoid Robotics'];

const containerVariants = {
  hidden: {},
  visible: {
    transition: { staggerChildren: 0.12, delayChildren: 0.2 },
  },
};

const wordVariants = {
  hidden: { opacity: 0, y: 30, filter: 'blur(8px)' },
  visible: {
    opacity: 1,
    y: 0,
    filter: 'blur(0px)',
    transition: { duration: 0.6, ease: [0.25, 0.46, 0.45, 0.94] as const },
  },
};

/* ─── Tech stack badges ─── */
const TECH_BADGES = [
  { name: 'Docusaurus', icon: '📖' },
  { name: 'FastAPI', icon: '⚡' },
  { name: 'Cohere', icon: '🧠' },
  { name: 'Qdrant', icon: '🔍' },
  { name: 'React 19', icon: '⚛️' },
];

/* ─── Particle background (browser-only) ─── */
function ParticleField() {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    let animId: number;
    let particles: { x: number; y: number; vx: number; vy: number; size: number; opacity: number }[] = [];

    const resize = () => {
      canvas.width = canvas.offsetWidth * window.devicePixelRatio;
      canvas.height = canvas.offsetHeight * window.devicePixelRatio;
      ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    };

    const init = () => {
      resize();
      const count = Math.min(60, Math.floor((canvas.offsetWidth * canvas.offsetHeight) / 15000));
      particles = Array.from({ length: count }, () => ({
        x: Math.random() * canvas.offsetWidth,
        y: Math.random() * canvas.offsetHeight,
        vx: (Math.random() - 0.5) * 0.3,
        vy: (Math.random() - 0.5) * 0.3,
        size: Math.random() * 2 + 0.5,
        opacity: Math.random() * 0.5 + 0.1,
      }));
    };

    const draw = () => {
      const w = canvas.offsetWidth;
      const h = canvas.offsetHeight;
      ctx.clearRect(0, 0, w, h);

      for (const p of particles) {
        p.x += p.vx;
        p.y += p.vy;
        if (p.x < 0) p.x = w;
        if (p.x > w) p.x = 0;
        if (p.y < 0) p.y = h;
        if (p.y > h) p.y = 0;

        ctx.beginPath();
        ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
        ctx.fillStyle = `rgba(0, 212, 255, ${p.opacity})`;
        ctx.fill();
      }

      // Draw connections
      for (let i = 0; i < particles.length; i++) {
        for (let j = i + 1; j < particles.length; j++) {
          const dx = particles[i].x - particles[j].x;
          const dy = particles[i].y - particles[j].y;
          const dist = Math.sqrt(dx * dx + dy * dy);
          if (dist < 120) {
            ctx.beginPath();
            ctx.moveTo(particles[i].x, particles[i].y);
            ctx.lineTo(particles[j].x, particles[j].y);
            ctx.strokeStyle = `rgba(0, 212, 255, ${0.08 * (1 - dist / 120)})`;
            ctx.lineWidth = 0.5;
            ctx.stroke();
          }
        }
      }

      animId = requestAnimationFrame(draw);
    };

    init();
    draw();
    window.addEventListener('resize', init);

    return () => {
      cancelAnimationFrame(animId);
      window.removeEventListener('resize', init);
    };
  }, []);

  return (
    <canvas
      ref={canvasRef}
      className={styles.particleCanvas}
      aria-hidden="true"
    />
  );
}

/* ─── Hero Section ─── */
function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  const shouldReduceMotion = useReducedMotion();
  const [isMounted, setIsMounted] = useState(false);

  useEffect(() => {
    setIsMounted(true);
  }, []);

  const handleTryTutor = useCallback(() => {
    if (!isBrowser) return;
    // Dispatch a custom event that Root.tsx can listen to
    window.dispatchEvent(new CustomEvent('open-chat-demo'));
  }, []);

  return (
    <header className={clsx(styles.heroBanner)}>
      {/* Particle background — browser only */}
      <BrowserOnly>{() => !shouldReduceMotion && <ParticleField />}</BrowserOnly>

      {/* Gradient orbs */}
      <div className={styles.heroOrb} />
      <div className={styles.heroOrb2} />

      <div className="container">
        <div className={styles.heroContent}>
          {/* Animated headline */}
          <motion.div
            className={styles.headlineWrapper}
            initial={shouldReduceMotion || !isMounted ? false : 'hidden'}
            animate={isMounted ? 'visible' : 'hidden'}
            variants={containerVariants}
          >
            <Heading as="h1" className={styles.heroTitle}>
              {headlineWords.map((word, i) => (
                <motion.span key={i} variants={wordVariants} className={styles.headlineWord}>
                  {word}
                </motion.span>
              ))}
            </Heading>
          </motion.div>

          {/* Subtitle */}
          <motion.p
            className={styles.heroSubtitle}
            initial={shouldReduceMotion || !isMounted ? false : { opacity: 0, y: 16 }}
            animate={isMounted ? { opacity: 1, y: 0 } : {}}
            transition={{ duration: 0.5, delay: 0.7 }}
          >
            An AI tutor that has read the entire textbook.
            <br className={styles.subtitleBreak} />
            <span className={styles.subtitleAccent}>Ask it anything.</span>
          </motion.p>

          {/* CTA buttons */}
          <motion.div
            className={styles.buttons}
            initial={shouldReduceMotion || !isMounted ? false : { opacity: 0, y: 16 }}
            animate={isMounted ? { opacity: 1, y: 0 } : {}}
            transition={{ duration: 0.5, delay: 0.9 }}
          >
            <Link
              className="button button--primary button--lg"
              to="/docs/Physical-AI-Humanoid-Robotics/"
            >
              Explore the Textbook
            </Link>
            <button
              onClick={handleTryTutor}
              className="button button--secondary button--lg"
            >
              Try the AI Tutor
            </button>
          </motion.div>

          {/* Conversation demo — centerpiece */}
          <motion.div
            className={styles.demoSection}
            initial={shouldReduceMotion || !isMounted ? false : { opacity: 0, y: 24 }}
            animate={isMounted ? { opacity: 1, y: 0 } : {}}
            transition={{ duration: 0.6, delay: 1.2 }}
          >
            <ConversationDemo onTryItself={handleTryTutor} />
          </motion.div>

          {/* Tech stack badges */}
          <motion.div
            className={styles.techBadges}
            initial={shouldReduceMotion || !isMounted ? false : { opacity: 0 }}
            animate={isMounted ? { opacity: 1 } : {}}
            transition={{ duration: 0.5, delay: 1.6 }}
          >
            {TECH_BADGES.map((badge) => (
              <motion.span
                key={badge.name}
                className={styles.techBadge}
                whileHover={shouldReduceMotion ? {} : { scale: 1.08, y: -2 }}
                transition={{ type: 'spring', stiffness: 400, damping: 17 }}
              >
                <span className={styles.badgeIcon}>{badge.icon}</span>
                {badge.name}
              </motion.span>
            ))}
          </motion.div>
        </div>
      </div>
    </header>
  );
}

/* ─── Module data ─── */
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
    description:
      'Master ROS 2 as the communication backbone for humanoid robots. Learn node communication, Python integration with rclpy, and robot description with URDF.',
    link: '/docs/Physical-AI-Humanoid-Robotics/ros2-nervous-system/',
    icon: '🧠',
  },
  {
    number: '02',
    title: 'Sensors & Perception',
    description:
      'Discover how humanoid robots sense their environment through cameras, depth sensors, IMUs, and advanced sensor fusion techniques for spatial awareness.',
    link: '/docs/Physical-AI-Humanoid-Robotics/sensors-perception/',
    icon: '👁️',
  },
  {
    number: '03',
    title: 'The AI-Robot Brain',
    description:
      'Explore NVIDIA Isaac platform for photorealistic simulation, hardware-accelerated perception, and specialized navigation for bipedal robots.',
    link: '/docs/Physical-AI-Humanoid-Robotics/isaac-ai-brain/',
    icon: '🤖',
  },
  {
    number: '04',
    title: 'Vision-Language-Action',
    description:
      'Build end-to-end autonomous behavior using foundation models, vision-language integration, and cognitive planning for humanoid intelligence.',
    link: '/docs/Physical-AI-Humanoid-Robotics/vision-language-action/',
    icon: '🎯',
  },
];

/* ─── Module card with scroll animation ─── */
function ModuleCard({
  number,
  title,
  description,
  link,
  icon,
  index,
}: ModuleItem & { index: number }) {
  const shouldReduceMotion = useReducedMotion();

  return (
    <motion.div
      className={clsx('col col--6', styles.moduleCard)}
      initial={shouldReduceMotion ? false : { opacity: 0, y: 30 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true, margin: '-60px' }}
      transition={{
        duration: 0.5,
        delay: index * 0.12,
        ease: [0.25, 0.46, 0.45, 0.94] as const,
      }}
    >
      <div className="card">
        <div className={styles.moduleNumber}>{number}</div>
        <div className="card__header">
          <h3>
            {icon} {title}
          </h3>
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
    </motion.div>
  );
}

/* ─── Modules Section ─── */
function HomepageModules() {
  const shouldReduceMotion = useReducedMotion();

  return (
    <section className={styles.modules}>
      <div className="container">
        <motion.div
          className={styles.sectionHeader}
          initial={shouldReduceMotion ? false : { opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true, margin: '-60px' }}
          transition={{ duration: 0.5 }}
        >
          <span className={styles.sectionBadge}>Curriculum</span>
          <h2 className={styles.sectionTitle}>Master Humanoid Robotics</h2>
          <p className={styles.sectionSubtitle}>
            A comprehensive journey from foundational robotic systems to cutting-edge
            AI integration. Each module builds upon the last.
          </p>
        </motion.div>
        <div className="row">
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}

/* ─── Feature data ─── */
type FeatureItem = {
  icon: string;
  title: string;
  description: string;
};

const FeatureList: FeatureItem[] = [
  {
    icon: '🤖',
    title: 'RAG-Powered AI Tutor',
    description:
      'Ask questions in natural language and get accurate answers sourced directly from the textbook — not generic AI hallucinations.',
  },
  {
    icon: '✨',
    title: 'Highlight & Ask',
    description:
      'Select any passage on any lesson page and ask the AI to explain it in context. The tutor knows exactly what you\'re reading.',
  },
  {
    icon: '📚',
    title: 'Source Citations',
    description:
      'Every AI response includes citations showing exactly which module and lesson the information came from. Trust, but verify.',
  },
  {
    icon: '💬',
    title: 'Conversation Memory',
    description:
      'Multi-turn conversations that remember context. Ask follow-ups like "Can you explain that differently?" and the tutor knows what you mean.',
  },
  {
    icon: '🧩',
    title: '4 Progressive Modules',
    description:
      'From ROS2 fundamentals to vision-language-action models. Each module builds on the last, taking you from beginner to advanced.',
  },
  {
    icon: '🌙',
    title: 'Dark Mode Native',
    description:
      'Built from the ground up for developers who prefer dark themes. Every component, every animation, every detail — designed for the dark side.',
  },
];

/* ─── Feature card with scroll animation ─── */
function FeatureCard({ icon, title, description, index }: FeatureItem & { index: number }) {
  const shouldReduceMotion = useReducedMotion();

  return (
    <motion.div
      className="col col--4"
      initial={shouldReduceMotion ? false : { opacity: 0, y: 24 }}
      whileInView={{ opacity: 1, y: 0 }}
      viewport={{ once: true, margin: '-40px' }}
      transition={{
        duration: 0.45,
        delay: index * 0.1,
        ease: [0.25, 0.46, 0.45, 0.94] as const,
      }}
    >
      <div className={styles.featureCard}>
        <span className={styles.featureIcon}>{icon}</span>
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </motion.div>
  );
}

/* ─── Features Section ─── */
function HomepageFeatures() {
  const shouldReduceMotion = useReducedMotion();

  return (
    <section className={styles.features}>
      <div className="container">
        <motion.div
          className={styles.sectionHeader}
          initial={shouldReduceMotion ? false : { opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true, margin: '-60px' }}
          transition={{ duration: 0.5 }}
        >
          <span className={styles.sectionBadge}>Why This Platform</span>
          <h2 className={styles.sectionTitle}>Learn Smarter, Build Faster</h2>
          <p className={styles.sectionSubtitle}>
            A textbook that talks back. Every feature designed to accelerate your
            journey into humanoid robotics.
          </p>
        </motion.div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <FeatureCard key={idx} {...props} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}

/* ─── Curriculum Timeline ─── */
function CurriculumTimeline() {
  const shouldReduceMotion = useReducedMotion();
  const lineRef = useRef<HTMLDivElement>(null);
  const [lineHeight, setLineHeight] = useState(0);

  useEffect(() => {
    if (lineRef.current) {
      setLineHeight(lineRef.current.scrollHeight);
    }
  }, []);

  return (
    <section className={styles.timelineSection}>
      <div className="container">
        <motion.div
          className={styles.sectionHeader}
          initial={shouldReduceMotion ? false : { opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true, margin: '-60px' }}
          transition={{ duration: 0.5 }}
        >
          <span className={styles.sectionBadge}>Learning Path</span>
          <h2 className={styles.sectionTitle}>Your Journey to Humanoid AI</h2>
          <p className={styles.sectionSubtitle}>
            Four modules. One complete skill set. From nervous system to full autonomy.
          </p>
        </motion.div>

        <div className={styles.timeline}>
          {/* Animated connecting line */}
          <motion.div
            ref={lineRef}
            className={styles.timelineLine}
            initial={shouldReduceMotion ? false : { scaleY: 0 }}
            whileInView={{ scaleY: 1 }}
            viewport={{ once: true, margin: '-40px' }}
            transition={{ duration: 1.2, ease: [0.25, 0.46, 0.45, 0.94] as const }}
            style={{ transformOrigin: 'top' }}
          />

          {ModuleList.map((mod, idx) => (
            <motion.div
              key={idx}
              className={styles.timelineItem}
              initial={shouldReduceMotion ? false : { opacity: 0, x: idx % 2 === 0 ? -30 : 30 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true, margin: '-40px' }}
              transition={{
                duration: 0.5,
                delay: 0.2 + idx * 0.15,
                ease: [0.25, 0.46, 0.45, 0.94] as const,
              }}
            >
              <div className={styles.timelineNode}>
                <span className={styles.timelineNumber}>{mod.number}</span>
              </div>
              <Link to={mod.link} className={styles.timelineCard}>
                <span className={styles.timelineIcon}>{mod.icon}</span>
                <h3>{mod.title}</h3>
                <p>{mod.description.slice(0, 100)}…</p>
              </Link>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  );
}

/* ─── CTA Section ─── */
function HomepageCTA() {
  const shouldReduceMotion = useReducedMotion();

  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <motion.h2
          className={styles.ctaTitle}
          initial={shouldReduceMotion ? false : { opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true, margin: '-40px' }}
          transition={{ duration: 0.5 }}
        >
          Ready to Build the Future?
        </motion.h2>
        <motion.p
          className={styles.ctaSubtitle}
          initial={shouldReduceMotion ? false : { opacity: 0, y: 16 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true, margin: '-40px' }}
          transition={{ duration: 0.5, delay: 0.1 }}
        >
          Start your journey into Physical AI and humanoid robotics today.
        </motion.p>
        <motion.div
          initial={shouldReduceMotion ? false : { opacity: 0, y: 16 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true, margin: '-40px' }}
          transition={{ duration: 0.5, delay: 0.2 }}
        >
          <Link
            className="button button--primary button--lg"
            to="/docs/Physical-AI-Humanoid-Robotics/"
          >
            Get Started Free →
          </Link>
        </motion.div>
      </div>
    </section>
  );
}

/* ─── Default export ─── */
export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Learn the complete technology stack for building intelligent humanoid robots with Physical AI. Features an AI tutor powered by RAG."
    >
      <HomepageHeader />
      <main>
        <CurriculumTimeline />
        <HomepageModules />
        <HomepageFeatures />
        <HomepageCTA />
      </main>
    </Layout>
  );
}
