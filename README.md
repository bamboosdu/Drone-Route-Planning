# 无人机区域覆盖路径规划 / Drone Area Coverage Path Planning

这是一个用于无人机区域覆盖路径规划的Python项目。该项目实现了基于Boustrophedon分解的区域覆盖路径规划算法，可以生成高效的无人机覆盖路径。

This is a Python project for drone area coverage path planning. It implements the Boustrophedon decomposition algorithm to generate efficient coverage paths for drones.

## 功能特点 / Features

- 支持多边形区域（包括带孔的多边形）的路径规划 / Supports path planning for polygonal areas (including polygons with holes)
- 实现了Boustrophedon分解算法 / Implements Boustrophedon decomposition algorithm
- 支持自定义路径间距 / Supports customizable path spacing
- 支持自定义路径角度 / Supports customizable path angle
- 提供可视化功能 / Provides visualization capabilities

## 系统要求 / System Requirements

- Python 3.8 或更高版本 / Python 3.8 or higher
- pip 包管理器 / pip package manager

## 依赖项 / Dependencies

- numpy >= 1.21.0
- shapely >= 2.0.0
- matplotlib >= 3.5.0

## 安装 / Installation

1. 克隆项目到本地 / Clone the project:
```bash
git clone [项目地址]
cd Drone-Route-Planning
```

2. 安装依赖 / Install dependencies:
```bash
pip install numpy>=1.21.0 shapely>=2.0.0 matplotlib>=3.5.0
```

## 使用方法 / Usage

1. 定义多边形区域 / Define polygon area:
```python
ext = [(0, 0), (4, 4), (0, 8), (-4, 4), (-9, 3)]  # 外边界 / outer boundary
holes = [[(0, 0), (0, 0), (0, 0), (0, 0)]]  # 内部孔洞（可选）/ inner holes (optional)
```

2. 创建AreaPolygon对象 / Create AreaPolygon object:
```python
polygon = AreaPolygon(ext, initial_pos=(-5, 10), interior=holes, ft=0.5, angle=30)
```

3. 生成覆盖路径 / Generate coverage path:
```python
path = polygon.get_area_coverage(origin=(0.0, 0.0))
```

4. 可视化结果 / Visualize results:
```python
import matplotlib.pyplot as plt
fig = plt.figure(1, dpi=90)
ax = fig.add_subplot(121)
plt.plot(*polygon.rP.exterior.xy)
plot_coords(ax, path)
plot_bounds(ax, path)
plot_line(ax, path)
plt.plot(*polygon.P.exterior.xy)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
```

## 参数说明 / Parameters

- `coordinates`: 多边形外边界坐标列表 / List of outer boundary coordinates
- `initial_pos`: 初始位置坐标 / Initial position coordinates
- `interior`: 内部孔洞坐标列表（可选）/ List of inner hole coordinates (optional)
- `ft`: 路径间距 / Path spacing
- `angle`: 路径角度（可选，默认基于最长边计算）/ Path angle (optional, defaults to calculation based on longest edge)

## 示例 / Examples

项目包含一个示例代码，展示了如何创建一个带孔的多边形并生成覆盖路径。运行`area_coverage.py`文件即可查看示例。

The project includes an example code that demonstrates how to create a polygon with holes and generate a coverage path. Run the `area_coverage.py` file to see the example.



## 许可证 / License

[待定 / To be determined]

## 贡献 / Contributing

欢迎提交问题和改进建议！ / Issues and pull requests are welcome!

