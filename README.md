# PCS - Point Cloud Shop

基于 Qt6 + VTK + PCL 的点云交互式可视化编辑工具。

![界面预览](https://pcs.creeper5820.com/2026-04-12-073903_hyprshot.png)

## 技术栈

- **GUI**: Qt6 (Widgets, OpenGLWidgets)
- **渲染**: VTK
- **点云处理**: PCL (io, common, filters)
- **语言**: C++23
- **构建**: CMake 3.22+
- **日志**: spdlog
- **UI 组件库**: creeper-qt

## 项目结构

```
src/
├── core/
│   ├── renderer/   # VTK 渲染引擎
│   ├── runtime/    # 异步事件队列
│   ├── units/      # 可视化单元（点云、坐标轴等）
│   ├── handles/    # 交互句柄
│   └── events/     # 事件系统
├── gui/
│   ├── app/                  # 应用入口
│   ├── visualization-window/ # 3D 可视化窗口
│   ├── vtk-window/           # VTK 渲染集成
│   ├── working-panel/        # 工作面板
│   └── side-toolbar/         # 侧边工具栏
└── utility/        # 通用工具（PIMPL、Qt 绑定、数学库等）
```

## 构建

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## TODO List

- [ ] 鼠标左键移动视角
- [ ] 体素点云或者网格化描述点云，半透明网格
- [ ] 按住特殊键放置参考点/参考网格（立体），放在点云表面上
- [ ] 按特殊按键切换至参考网格移动模式，颜色加深，利用方向键、空格/Shift 移动网格，从表面移动到指定位置
- [ ] 移动时框选的空间也要变化，用另一种颜色表明，但都是透明
- [ ] 第一视角移动，可以切换点云缩放率和网格缩放率
- [ ] 三个确定网格一个立方体区间
- [ ] 不同形状的选择空间
- [ ] 几个选择的点云簇可以缓存，合并处理（缓存区域，类似于剪切板）
- [ ] 测量距离
- [ ] 添加若干空间盒子来选择点云
- [ ] 导向球，用于拖动某个空间点，xyz 方向移动
- [ ] 空间几何体的角点也可以配合导向球
- [ ] 再配合方格体素化，选择格子点
- [ ] 支持 TF 树，动态插入变换
