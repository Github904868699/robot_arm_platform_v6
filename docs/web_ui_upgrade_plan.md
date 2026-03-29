# web_ui_upgrade_plan.md

## 为什么先做 i18n + 视觉升级 + mock 数据

在后端与硬件接口尚未稳定前，先完成国际化与产品化首页骨架，可以并行推进交互与信息架构，减少后续返工。

## 为什么前端不直接碰底层协议

前端只表达产品语义（状态、模式、任务），不应感知 bus/protocol/node_id/endianness。

底层协议差异应封装在 web bridge + core + hardware 层。

## 当前首页视觉目标

- 深色高端控制台气质
- 顶部 segmented mode switch
- 左侧 telemetry 卡片化
- 中央主舞台（3D placeholder）
- 右侧功能卡主次分层
- 保持克制的动效与层级

## 后续如何接入 web bridge

1. 保持前端通过高层 API 获取状态与执行命令。
2. 首先替换 mock 数据源为 mock API。
3. 再接入真实 web bridge：
   - get robot state
   - get joints
   - enable/disable robot
   - get mode
   - future task execution API
4. 全程不向前端暴露协议细节。
