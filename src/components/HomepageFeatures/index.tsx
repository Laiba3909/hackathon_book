import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  src: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical Embodiment',
    src: '/img/robot11.webp',
    description: (
      <>
        Understand the mechanics and design principles behind creating robots that can interact with the real world.
      </>
    ),
  },
  {
    title: 'Advanced AI Integration',
    src: '/img/=robot12.webp',
    description: (
      <>
        Explore how cutting-edge AI, from deep learning to reinforcement learning, drives robotic intelligence.
      </>
    ),
  },
  {
    title: 'Humanoid Interaction',
    src: '/img/robot13.jpg',
    description: (
      <>
        Delve into the complexities of human-robot interaction, communication, and collaborative tasks.
      </>
    ),
  },
  {
    title: 'Future of Robotics',
    src: '/img/robot14.avif',
    description: (
      <>
        Gaze into the future of robotics, from ethical considerations to the next generation of autonomous systems.
      </>
    ),
  },
];

function Feature({title, src, description}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
      <div className="text--center">
        <img src={src} className={styles.featureSvg} alt={title} />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
