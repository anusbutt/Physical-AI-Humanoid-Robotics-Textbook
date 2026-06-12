/**
 * ConversationDemo — self-playing, looping mock chat demonstration
 * Shows the AI tutor in action without requiring user interaction
 *
 * State machine: typing_question → thinking → typing_answer → pausing → (next)
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { motion, AnimatePresence } from 'motion/react';
import { TypingDots } from '../TypingDots';
import styles from './ConversationDemo.module.css';

/* ── Q&A pairs extracted from actual course content ── */
const QA_PAIRS = [
  {
    question: 'What is ROS2?',
    answer:
      'ROS2 is an open-source middleware framework that serves as the communication backbone for modern robots. It enables distributed systems architecture, letting sensors, motors, and algorithms talk to each other efficiently across different computers — like a nervous system for robots.',
    source: { module: 'Module 1', lesson: 'ROS2 Fundamentals' },
  },
  {
    question: "What's the difference between a node and a topic?",
    answer:
      'A node is an independent software component that performs a specific task — like processing camera data or controlling motors. A topic is a named channel through which nodes exchange messages. Nodes publish data to topics or subscribe to them to receive data from other nodes.',
    source: { module: 'Module 1', lesson: 'Nodes, Topics & Services' },
  },
  {
    question: 'How does sensor fusion work?',
    answer:
      'Sensor fusion combines data from multiple sensors — cameras, LiDAR, IMUs — to create a more accurate and reliable understanding of the environment. By merging complementary data sources, the robot can overcome the limitations of any single sensor and build a coherent world model.',
    source: { module: 'Module 2', lesson: 'Sensor Fusion Techniques' },
  },
];

/* ── Timing constants (ms) ── */
const TYPE_SPEED = 35;          // ms per character for question
const TYPE_ANSWER_SPEED = 12;   // ms per character for answer (faster)
const THINKING_DELAY = 800;     // ms showing typing dots
const PAUSE_AFTER_ANSWER = 3500; // ms to read the completed exchange
const FADE_OUT_DURATION = 400;  // ms to fade between exchanges

type Phase =
  | 'typing_question'
  | 'thinking'
  | 'typing_answer'
  | 'pausing'
  | 'fading_out';

const ConversationDemo: React.FC<{ onTryItself?: () => void }> = ({
  onTryItself,
}) => {
  const [pairIndex, setPairIndex] = useState(0);
  const [phase, setPhase] = useState<Phase>('typing_question');
  const [displayedQuestion, setDisplayedQuestion] = useState('');
  const [displayedAnswer, setDisplayedAnswer] = useState('');
  const [isReducedMotion, setIsReducedMotion] = useState(false);
  const timeoutsRef = useRef<ReturnType<typeof setTimeout>[]>([]);

  // Check for reduced motion preference
  useEffect(() => {
    const mq = window.matchMedia('(prefers-reduced-motion: reduce)');
    setIsReducedMotion(mq.matches);
    const handler = (e: MediaQueryListEvent) => setIsReducedMotion(e.matches);
    mq.addEventListener('change', handler);
    return () => mq.removeEventListener('change', handler);
  }, []);

  // Cleanup timeouts on unmount
  useEffect(() => {
    return () => {
      timeoutsRef.current.forEach(clearTimeout);
    };
  }, []);

  const currentPair = QA_PAIRS[pairIndex];

  /* ── Phase machine ── */
  useEffect(() => {
    timeoutsRef.current.forEach(clearTimeout);
    timeoutsRef.current = [];

    if (isReducedMotion) {
      // Reduced motion: show everything at once, pause, then cycle
      setDisplayedQuestion(currentPair.question);
      setDisplayedAnswer(currentPair.answer);
      setPhase('pausing');
      const t = setTimeout(() => {
        setPhase('fading_out');
        setTimeout(() => {
          setDisplayedQuestion('');
          setDisplayedAnswer('');
          setPairIndex((i) => (i + 1) % QA_PAIRS.length);
          setPhase('pausing');
        }, FADE_OUT_DURATION);
      }, PAUSE_AFTER_ANSWER);
      timeoutsRef.current.push(t);
      return;
    }

    if (phase === 'typing_question') {
      setDisplayedQuestion('');
      setDisplayedAnswer('');
      let charIdx = 0;
      const type = () => {
        charIdx++;
        setDisplayedQuestion(currentPair.question.slice(0, charIdx));
        if (charIdx < currentPair.question.length) {
          const t = setTimeout(type, TYPE_SPEED);
          timeoutsRef.current.push(t);
        } else {
          const t = setTimeout(() => setPhase('thinking'), 300);
          timeoutsRef.current.push(t);
        }
      };
      const initial = setTimeout(type, 400);
      timeoutsRef.current.push(initial);
    }

    if (phase === 'thinking') {
      const t = setTimeout(() => setPhase('typing_answer'), THINKING_DELAY);
      timeoutsRef.current.push(t);
    }

    if (phase === 'typing_answer') {
      setDisplayedAnswer('');
      let charIdx = 0;
      const type = () => {
        charIdx++;
        setDisplayedAnswer(currentPair.answer.slice(0, charIdx));
        if (charIdx < currentPair.answer.length) {
          const t = setTimeout(type, TYPE_ANSWER_SPEED);
          timeoutsRef.current.push(t);
        } else {
          const t = setTimeout(() => setPhase('pausing'), 600);
          timeoutsRef.current.push(t);
        }
      };
      const initial = setTimeout(type, 200);
      timeoutsRef.current.push(initial);
    }

    if (phase === 'pausing') {
      const t = setTimeout(() => setPhase('fading_out'), PAUSE_AFTER_ANSWER);
      timeoutsRef.current.push(t);
    }

    if (phase === 'fading_out') {
      const t = setTimeout(() => {
        setDisplayedQuestion('');
        setDisplayedAnswer('');
        setPairIndex((i) => (i + 1) % QA_PAIRS.length);
        setPhase('typing_question');
      }, FADE_OUT_DURATION);
      timeoutsRef.current.push(t);
    }
  }, [phase, pairIndex, currentPair, isReducedMotion]);

  return (
    <div className={styles.demoWrapper}>
      <div className={styles.chatWindow}>
        {/* Window header bar */}
        <div className={styles.windowHeader}>
          <div className={styles.windowDots}>
            <span className={styles.windowDot} />
            <span className={styles.windowDot} />
            <span className={styles.windowDot} />
          </div>
          <span className={styles.windowTitle}>AI Tutor — Live Demo</span>
        </div>

        {/* Messages area */}
        <div className={styles.messagesArea}>
          {/* User question */}
          <AnimatePresence>
            {displayedQuestion && (
              <motion.div
                initial={isReducedMotion ? false : { opacity: 0, y: 8 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.2 }}
                className={styles.userBubble}
              >
                <span className={styles.userLabel}>You</span>
                <p className={styles.messageText}>
                  {displayedQuestion}
                  {phase === 'typing_question' && (
                    <span className={styles.cursor}>|</span>
                  )}
                </p>
              </motion.div>
            )}
          </AnimatePresence>

          {/* Thinking indicator */}
          <AnimatePresence>
            {phase === 'thinking' && (
              <motion.div
                initial={isReducedMotion ? false : { opacity: 0, scale: 0.9 }}
                animate={{ opacity: 1, scale: 1 }}
                exit={{ opacity: 0, scale: 0.9 }}
                transition={{ duration: 0.2 }}
                className={styles.aiBubble}
              >
                <span className={styles.aiLabel}>AI Tutor</span>
                <TypingDots compact label="" />
              </motion.div>
            )}
          </AnimatePresence>

          {/* AI answer */}
          <AnimatePresence>
            {displayedAnswer && (
              <motion.div
                initial={isReducedMotion ? false : { opacity: 0, y: 8 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.25 }}
                className={styles.aiBubble}
              >
                <span className={styles.aiLabel}>AI Tutor</span>
                <p className={styles.messageText}>
                  {displayedAnswer}
                  {(phase === 'typing_answer' || phase === 'thinking') && (
                    <span className={styles.cursor}>|</span>
                  )}
                </p>
                {/* Source citation chip */}
                {phase === 'pausing' && (
                  <motion.div
                    initial={isReducedMotion ? false : { opacity: 0, y: 4 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.3, delay: 0.2 }}
                    className={styles.sourceChip}
                  >
                    📚 {currentPair.source.module} · {currentPair.source.lesson}
                  </motion.div>
                )}
              </motion.div>
            )}
          </AnimatePresence>
        </div>
      </div>

      {/* CTA below demo */}
      {onTryItself && (
        <motion.button
          onClick={onTryItself}
          className={styles.tryButton}
          whileHover={{ scale: 1.03 }}
          whileTap={{ scale: 0.98 }}
          initial={isReducedMotion ? false : { opacity: 0, y: 12 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.4, delay: 0.6 }}
        >
          Try it yourself →
        </motion.button>
      )}
    </div>
  );
};

export default ConversationDemo;
