# PCS - Point Cloud Shop

基于 Qt6 + VTK + PCL 的桌面端点云可视化与处理工具，支持模型转点云、点云处理、PNG 地图编辑与 ROS 地图导出。

<table border="0">
  <tr>
    <td width="60%" align="center">
      <img src="https://pcs.creeper5820.com/2026-04-21-091132_hyprshot.png"/>
      <sub>软件运行界面</sub>
    </td>
    <td width="35%" align="center">
      <img src="https://pcs.creeper5820.com/IMG_20250510_173253.jpg"/>
      <sub>哨兵祈祷中 ......</sub>
    </td>
  </tr>
</table>

## 快速体验

将 AppImage 下载至本地后，赋予执行权限即可使用，某些系统会提示没有 FUSE，可以使用包管理下载

- 正式版下载：[「正式版发布页」](https://github.com/creeper5820/pcs/releases/latest)
- 开发版下载：[「开发版发布页」](https://github.com/creeper5820/pcs/releases/tag/nightly)

## 当前能力

- 资产导入
  - `.obj` 模型文件
  - `.pcd` 点云文件
- 模型处理
  - OBJ -> 点云（密度、采样间距、单位缩放、最大点数）
- 点云处理
  - 点云 -> PNG 地图（分辨率、有效点云数、高度差、影响半径、Z 区间）
  - 聚类：保留最大簇 / 去除较小簇
  - 范围截取（X/Y/Z）+ 截取区域可视化
  - 平移旋转（含 Pivot）
  - 点云复制、保存为 `.pcd`
  - 点云颜色 RGBA 调整、坐标系显示开关
- PNG 地图处理
  - 编辑模式：自由 / 线 / 点 / 擦除（可调线宽、点大小、擦除大小）
  - 设置导出坐标系原点、Yaw 调整
  - 导出镜像选项：左右镜像 / 上下镜像 / 不镜像
  - 导出 ROS 地图目录（`map.png` + `map.yaml`）

## 技术栈

- **语言**: `C++23`
- **GUI**: `Qt6`
- **渲染**: `VTK`
- **点云处理**: `PCL`
- **日志**: `spdlog`
- **UI 组件库**: [`creeper-qt`](https://github.com/creeper5820/creeper-qt)

## 构建与运行

### 1) 常规构建

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 12
```

### 2) 启动

```bash
./build/pcs
```

## 测试

默认不编译测试。开启方式：

```bash
cmake -S . -B build-test -DPCS_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-test --parallel
ctest --test-dir build-test --output-on-failure
```

## Linux AppImage 打包

项目已内置 Linux 打包脚本：

```bash
cmake -S . -B build/appimage -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build build/appimage --parallel
./deploy/deploy-linux.sh build/appimage pcs release-appimage
```

产物默认位于：`release-appimage/pcs-x86_64.AppImage`
