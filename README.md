# Validate_Kalman_Filter

通过可配置的仿真真值，验证自瞄场景下 EKF 的建模正确性与参数收敛性。

## 1. 设计初衷

视觉自瞄中，验证 EKF 模型和参数通常依赖电控联调，且会严重受到检测精度的影响。
本项目通过生成**无检测误差的仿真观测**，把“检测问题”和“滤波问题”解耦，从而可以专注于：

- 验证不同运动建模是否正确；
- 验证不同过程/观测噪声参数是否收敛；
- 通过 NIS / NEES 一致性指标定量评估滤波器。

## 2. 整体架构

项目由三部分组成，彼此解耦、可独立替换：

```text
                 +-----------------------+
                 |     Motion Generator  |
                 |  TranslationMotion    |  平移策略（直线 / 随机）
                 |  SpinGenerator        |  自旋（正弦叠加）
                 |  Robot                |  刚体模型 + 装甲板观测
                 +-----------+-----------+
                             | GeneratorState（真值 + 观测）
                             v
                 +-----------------------+
                 |   Kalman Filter       |
                 |  ArmorModel           |  运动/观测模型与雅可比
                 |  EKF                  |  通用扩展卡尔曼滤波
                 |  Kalman               |  时序调度
                 +-----------+-----------+
                             | 状态估计 / NIS / NEES
                             v
                 +-----------------------+
                 |   Validation          |
                 |  ekf_test / PlotJuggler
                 +-----------------------+
```

### 2.1 目录结构

```text
tools/                          # 无状态通用工具
  math_tools/                   # 角度、时间、坐标转换
  sine_function/                # 单频正弦角速度模型
  robot_state/                  # 全局共享的状态/观测下标约定（header-only）
  plotjuggler/                  # UDP 数据可视化客户端
  exiter/                       # Ctrl-C 优雅退出
tasks/
  motion_generator/
    generator/                  # 后台线程调度，输出真值快照
    translate/                  # 平移策略：接口 + 直线/随机实现
    spin/                       # 自旋运动
    robot/                      # 刚体模型与装甲板观测
  kalman_filter_fitter/
    ekf/                        # 与模型无关的通用 EKF
    model/                      # 装甲板运动/观测模型
    kalman/                     # 时序调度（拟合器）
tests/
  generator_test.cpp            # 真值可视化
  ekf_test.cpp                  # EKF 验证与统计
```

### 2.2 运动状态生成器

- **自旋部分**：由若干个 `A·sin(f·t+φ)+x` 形式的角速度函数叠加，解析求导得到角加速度、
  积分得到累计角度。见 `tools::SineFunction` 与 `motion_generator::SpinGenerator`。
- **平移部分**：采用策略模式，见 `motion_generator::TranslationMotion`。
  - `LineTranslation`：两点之间确定性往返，端点速度为零，速度曲线为半个余弦周期；
  - `RandomTranslation`：随机目标速度 + 加速度/jerk 约束后积分，可复现且物理连续。
- **机器人模型**：`motion_generator::Robot` 把中心平移与自旋叠加，计算四块装甲板的
  球坐标观测，并按可见性阈值选择被锁定的装甲板。

角度统一使用弧度，输出的 yaw、pitch 和装甲板角度归一化到 `[-π, π)`；
角速度单位为 rad/s，角加速度单位为 rad/s²。`spin.phi_lists`、
`robot.detect_min_threshold`、`robot.detect_max_threshold` 均使用弧度。

### 2.3 EKF 拟合器

- `kalman_filter::EKF`：只依赖调用方传入的预测/观测函数与雅可比，与具体机器人无关；
  使用 Joseph 形式更新协方差，并内置 NIS / NEES 统计。
- `kalman_filter::ArmorModel`：装甲板机器人的运动/观测模型，提供初始状态、初始协方差、
  过程噪声、观测函数与观测雅可比。
- `kalman_filter::Kalman`：组合上述两者，完成“首帧初始化 → 时域预测 → 后验更新 →
  固定时长纯预测”的完整流程。

所有状态下标集中在 `tools/robot_state/robot_state.hpp`，避免在代码中散落魔术数字。

## 3. 构建与运行

依赖：CMake ≥ 3.8、C++17、Eigen3、OpenCV、yaml-cpp、nlohmann_json、Threads。

```bash
cmake -S . -B build
cmake --build build -j
```

运行示例：

```bash
# 可视化真值（需先启动 PlotJuggler 并监听 UDP 9870）
./build/generator_test --config-path=configs/motion_generator_test.yaml

# 验证 EKF，默认运行 60 秒
./build/ekf_test --config-path=configs/motion_generator_test.yaml --duration=60
```

`ekf_test` 会打印各状态分量的 RMSE 以及 NIS / NEES 均值。

## 4. 配置

配置集中在 `configs/motion_generator_test.yaml`，主要分为：

- `generator.update_rate`：后台真值更新周期（ms）；
- `robot.*`：装甲板数量、可见性阈值、原始状态到滤波状态的映射、初始状态；
- `spin.*`：正弦叠加个数与参数、物理尺寸；
- `translation.*`：平移模式与直线/随机参数；
- `kalman_filter.*`：`P`、`Q`、`R`、`predict_time` 与物理尺寸。

修改配置即可构造不同的验证场景，无需改动代码。

## 5. API 文档

公共接口均带有 Doxygen 注释。安装 `doxygen` 后可用以下任一方式生成 HTML 文档：

```bash
doxygen Doxyfile
# 或在构建目录中：
cmake --build build --target docs
```

输出位于 `docs/doxygen/html/index.html`。
