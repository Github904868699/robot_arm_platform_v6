export const zhControl = {
  modes: {
    control: '控制',
    teach: '示教',
    visual: '可视化',
    intelligent: '智能',
  },
  stage: {
    title: '3D 机器人主视区',
    subtitle: '产品主舞台占位（mock）',
    badge: 'Viewport',
  },
  cards: {
    cartesian: {
      title: '笛卡尔控制卡',
      desc: '预留高层控制入口，不直连底层协议。',
    },
    teach: {
      title: '示教卡',
      desc: '点位记录与编辑流程占位。',
    },
    motion: {
      title: '运动序列卡',
      desc: '任务步骤编排与执行占位。',
    },
    io: {
      title: 'IO 占位卡',
      desc: '预留 SetIO / WaitIO 扩展。',
    },
    teachTypes: {
      title: '示教动作类型',
      items: ['MoveJ', 'MoveL', 'MoveC', 'Wait', 'SetIO'],
    },
    sequence: {
      title: 'Teach 序列入口',
      empty: '暂无步骤',
    },
  },
};
