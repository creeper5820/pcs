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

## 基本工作流

### OBJ -> PCD

1. 在左侧工作区打开一个 `.obj` 模型文件。
2. 选中该模型资产后，右侧操作面板会显示 `转换为点云`。
3. 根据需要调整参数：
   - `Density`：采样密度，默认 `10`
   - `Sample Distance`：采样步长
   - `Unit Scale`：模型单位缩放
   - `Max Points`：点数上限，`0` 表示不限制
4. 点击 `转换为点云`。
5. 转换通过异步事件执行，按钮会进入 `生成中...` 状态，完成后自动生成并选中新点云资产。

说明：
- 同一个模型重复生成点云时，会优先覆盖同源生成结果，而不是无限新增重复资产。
- 生成后的点云可以继续调整整体颜色，也可以另存为 `.pcd`。

### PCD -> PNG

1. 打开或生成一个 `.pcd` 点云资产。
2. 选中点云后，右侧操作面板会显示 `生成 PNG 地图`。
3. 根据需要调整参数：
   - `Resolution`：地图分辨率（米/像素）
   - `Points Limit`：单像素参与统计的点数下限
   - `Height Limit`：高度阈值
   - `Influence Radius`：点对像素的影响半径
   - `Z Area [start, end]`：高度区间，默认 `0 ~ 1`
4. 点击 `生成 PNG 地图`，系统会异步执行点云转地图事件。
5. 完成后会生成一个 PNG 地图资产，并自动刷新到资产列表中。

说明：
- 同一个点云重复生成 PNG 地图时，会优先覆盖同源生成结果。
- 生成后的 PNG 地图支持继续编辑，包括画线、点绘制、擦除、设置导出坐标系原点、调整 `Yaw`，以及导出 ROS 地图目录。

<details>
<summary>点击展开算法原理</summary>

实际实现不是简单的“直接投影后统计点数”，而是分成两轮处理：预过滤 + 局部高度变化判定。

1. 先把输入点云转成 `pcl::PointCloud<pcl::PointXYZ>`。
2. 对点云做两步预处理：
   - `StatisticalOutlierRemoval` 去离群点，参数固定为 `MeanK=20`、`StddevMulThresh=0.5`
   - `VoxelGrid` 体素降采样，体素边长直接使用当前 `Resolution`
3. 记录原始点云的最小 `z` 作为基准高度，然后把用户输入的 `Z Area [start, end]` 转成实际高度区间：
   - `area_start = original_min_z + z_area_start`
   - `area_end = original_min_z + z_area_end`
4. 用过滤后的点云重新计算平面范围，按 `Resolution` 建立二维栅格，地图尺寸来自过滤后点云的 `x/y` 包围盒。
5. 遍历过滤后的点，只处理落在上述 Z 区间内的点。每个点不会只落到单个像素，而是会先按一个固定采样半径扩散到周围像素：
   - 这个采样半径固定按 `0.1 / resolution` 转成像素，不直接使用 `Influence Radius`
   - 扩散区域是圆形邻域
6. 每个像素维护一个 `height_table`，里面存的是该像素命中的离散高度值，精度为厘米级：
   - 高度会先做 `round(height * 100)` 再存入集合
   - 因此这里统计的是“唯一高度层数”和“最大高度跨度”，不是简单累加点数
7. 第一轮遍历结束后，只对被命中过的像素做障碍判定。某像素会被判为“障碍种子”，必须同时满足：
   - `height_table.size() >= Points Limit`
   - `max(height_table) - min(height_table) >= Height Limit`
8. 对于每个障碍种子，再按 `Influence Radius / Resolution` 转成像素半径，在圆形邻域内把像素值统一写成 `0`。
9. 最终输出单通道 PNG：
   - 默认像素值是 `255`
   - 被障碍种子影响到的区域写成 `0`

因此这套算法更接近“基于局部高度离散度生成障碍图”：
- `Points Limit` 控制一个像素里至少要出现多少个离散高度层
- `Height Limit` 控制这些高度层的最大跨度是否足够大
- `Influence Radius` 只在最终障碍膨胀阶段起作用
- `plane_z` 最终会取 `area_start`
</details>

### PNG 导出（ROS）

1. 选中 PNG 地图资产。
2. 在右侧面板中：
   - 调整 `Yaw`
   - 点击 `设置坐标系原点 ...`，进入拾取模式，在地图上点取原点
3. 点击 `导出`。
4. 选择目标目录后，程序会创建导出文件夹，输出：
   - `map.png`
   - `map.yaml`

`map.yaml` 中会包含 ROS 全局规划常用字段：
- `image`
- `resolution`
- `origin`
- `negate`
- `occupied_thresh`
- `free_thresh`

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
