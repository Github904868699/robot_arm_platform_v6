export const enControl = {
  modes: {
    control: 'Control',
    teach: 'Teach',
    visual: 'Visual',
    intelligent: 'Intelligent',
  },
  stage: {
    title: '3D Robot Main Stage',
    subtitle: 'Product-stage placeholder (mock)',
    badge: 'Viewport',
  },
  cards: {
    cartesian: {
      title: 'Cartesian Control Card',
      desc: 'Reserved high-level controls without protocol coupling.',
    },
    teach: {
      title: 'Teach Card',
      desc: 'Placeholder for point record and editing workflow.',
    },
    motion: {
      title: 'Motion Sequence Card',
      desc: 'Placeholder for task timeline and execution.',
    },
    io: {
      title: 'IO Placeholder Card',
      desc: 'Reserved for SetIO / WaitIO extensions.',
    },
    teachTypes: {
      title: 'Teach Action Types',
      items: ['MoveJ', 'MoveL', 'MoveC', 'Wait', 'SetIO'],
    },
    sequence: {
      title: 'Teach Sequence Entry',
      empty: 'No steps',
    },
  },
};
