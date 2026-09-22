# 平移运动模拟设计建议

## 1. 设计目标

本项目的目标不是只生成一条看起来合理的轨迹，而是通过高度可配置的运动场景，验证不同 EKF 模型和参数组合的表现。

因此，平移运动生成器需要同时满足以下要求：

- 能生成可复现的确定性轨迹，便于定位模型和代码问题；
- 能生成具有随机性的复杂轨迹，便于评估 EKF 的鲁棒性；
- 运动状态应满足基本的物理连续性，不能每帧直接随机跳变位置；
- 真值轨迹和 EKF 预测模型应允许存在可控的模型失配；
- 相同配置和随机种子应能重复生成相同轨迹。

“高度自定义”和“随机运动”并不矛盾。更合适的方式是让用户配置随机过程的范围、分布和运动约束，而不是直接对每一帧的位置添加随机数。

## 2. 平移与自旋的职责划分

机器人应先计算自身中心的平移状态，再叠加装甲板相对于中心的自旋状态：

```text
center_position(t) = translation(t)
armor_position(t) = center_position(t) + Rz(yaw(t)) * armor_offset
```

平移模块只负责机器人中心的：

- 位置；
- 速度；
- 加速度；
- 必要时的 jerk（加加速度）。

自旋模块只负责：

- 自旋角度；
- 角速度；
- 角加速度；
- 装甲板绕旋转中心的相对位置。

不要给每块装甲板分别生成一套平移轨迹，否则不同装甲板之间会失去刚体约束。

## 3. 确定性轨迹和随机轨迹

建议保留三类平移模式。

### 3.1 确定性轨迹

用于验证运动方程、坐标变换和 EKF 实现是否正确：

- 匀速运动；
- 匀加速运动；
- 正弦横移；
- 圆周或弧线运动；
- 急停和折返。

例如正弦横移：

```text
y(t)  = y0 + A sin(wt + phi)
vy(t) = A w cos(wt + phi)
ay(t) = -A w^2 sin(wt + phi)
```

这类轨迹适合直接根据绝对时间 `t` 计算，不需要数值积分。

### 3.2 脚本轨迹

由用户明确指定每个阶段的目标速度和持续时间，例如：

```yaml
segments:
  - duration: 1.0
    target_velocity: [1.0, 0.0, 0.0]
  - duration: 0.8
    target_velocity: [0.0, 1.5, 0.0]
  - duration: 0.6
    target_velocity: [0.0, 0.0, 0.0]
```

脚本轨迹适合复现某个具体工况，也适合定位某一次发散的原因。

### 3.3 随机轨迹

随机轨迹适合批量测试和调参，但应随机生成“运动意图”，而不是随机生成位置。

推荐流程：

```text
随机目标速度
        ->
加速度限制
        ->
jerk 限制
        ->
积分得到速度和位置
```

随机模式应至少支持以下配置项：

- 速度上下限；
- 最大加速度；
- 最大 jerk；
- 目标速度保持时间范围；
- 目标速度分布；
- 急停概率；
- 反向概率；
- 随机种子。

## 4. 推荐的随机平移模型

维护如下真值状态：

```cpp
struct TranslationState
{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
};
```

每隔一段时间生成新的目标速度 `target_velocity`，并使当前速度平滑地接近目标速度：

```text
a_command = (target_velocity - velocity) / response_time
a_command = clamp_norm(a_command, max_acceleration)

acceleration += clamp_norm(
    a_command - acceleration,
    max_jerk * dt
)

position += velocity * dt + 0.5 * acceleration * dt^2
velocity += acceleration * dt
```

该模型可以自然生成：

- 匀速段；
- 加速和减速；
- 急停和重新启动；
- 斜向运动；
- 连续变向；
- 速度和加速度受限的随机机动。

相比每帧给位置加入随机数，这种方式具有更好的连续性，也更适合测试 EKF 的速度和加速度状态。

## 5. `evaluate(t)` 和 `advance(dt)` 的取舍

两种接口都需要保留，但适用于不同类型的运动。

### 5.1 解析运动使用 `evaluate(t)`

对于正弦、匀速、圆周等可以写成时间函数的轨迹，推荐：

```cpp
TranslationState evaluate(double t) const;
```

特点：

- 没有积分误差；
- 重新取样不会改变轨迹；
- 可以直接计算位置、速度和加速度；
- 适合暂停、重复播放和跳转到任意时间点。

### 5.2 随机和受约束运动使用 `advance(dt)`

对于随机目标速度、加速度限制、jerk 限制和事件驱动的运动，必须按时间顺序推进内部状态：

```cpp
void advance(double dt);
const TranslationState& state() const;
```

此类生成器内部需要保存：

- 当前位置；
- 当前速度；
- 当前加速度；
- 当前目标速度；
- 当前目标速度的剩余保持时间；
- 随机数引擎和种子。

不能每次只根据总时间重新计算随机运动，否则随机事件和状态连续性无法正确表达。

### 5.3 推荐的统一抽象

可以在运动生成器层统一输出 `RobotState`，而不强制所有运动都使用同一套内部实现：

```cpp
class AnalyticMotion
{
public:
    virtual ~AnalyticMotion() = default;
    virtual TranslationState evaluate(double t) const = 0;
};

class StatefulMotion
{
public:
    virtual ~StatefulMotion() = default;
    virtual void advance(double dt) = 0;
    virtual const TranslationState& state() const = 0;
};
```

上层机器人只关心当前的中心状态和装甲板观测，不关心该状态是通过解析公式还是数值积分得到的。

## 6. 仿真时钟

仿真应使用独立的、可控的仿真时间：

```cpp
double simulation_time = 0.0;
const double dt = 0.001;

while (simulation_time < total_time)
{
    generator.advance(dt);
    simulation_time += dt;

    if (should_measure(simulation_time))
    {
        auto truth = generator.state();
        auto observation = generator.observe();
        ekf.update(observation);
    }
}
```

建议将以下频率分开：

```text
真值内部更新频率：例如 1 kHz
观测频率：例如 100~200 Hz
EKF 预测频率：根据待验证模型单独配置
```

即使当前没有观测，真值也应该继续运动。不要让“是否产生观测”决定机器人状态是否更新。

不建议让运动生成器直接依赖 `std::chrono::steady_clock`。现实时间会受到线程调度和程序运行速度影响，导致同一组参数无法稳定复现。真实时间可以用于实时演示，但调参和回归测试应使用显式仿真时间。

## 7. 推荐配置结构

```yaml
translation:
  mode: stochastic_target_velocity

  initial:
    position: [0.0, 0.0, 0.0]
    velocity: [0.0, 0.0, 0.0]

  constraints:
    velocity_min: [-3.0, -2.0, 0.0]
    velocity_max: [ 3.0,  2.0, 0.0]
    max_acceleration: 4.0
    max_jerk: 12.0

  command:
    hold_time_min: 0.4
    hold_time_max: 2.0
    response_time: 0.25
    stop_probability: 0.15
    reverse_probability: 0.20

  random:
    seed: 42
```

之后可以进一步开放随机分布：

```yaml
velocity_distribution: gaussian
velocity_mean: [0.0, 0.0, 0.0]
velocity_stddev: [1.5, 1.0, 0.0]
acceleration_noise_stddev: 0.2
```

默认应使用固定种子。批量调参时再自动遍历多个种子。

## 8. EKF 调参方式

真值生成模型不应和 EKF 预测模型完全相同，否则测试结果会过于理想。

推荐使用以下顺序：

1. 使用匀速真值验证 EKF 方程和坐标变换；
2. 使用匀加速和正弦运动验证速度、加速度状态；
3. 使用脚本轨迹定位急停、变向等具体问题；
4. 使用多个随机种子评估整体鲁棒性；
5. 最后加入观测噪声、延迟、掉帧和离群点。

每组 EKF 参数不要只看单次轨迹。建议统计：

- 位置 RMSE；
- 速度 RMSE；
- 最大位置误差；
- 95% 分位误差；
- 收敛时间；
- 发散次数；
- NIS / NEES。

最终建议采用以下整体结构：

```text
可配置确定性轨迹
        +
可复现的随机运动生成器
        +
固定仿真时钟
        +
可配置的观测噪声和异常
        +
多随机种子的统计评估
```

这样既能保留项目原本的高度自定义能力，也能用随机运动检验 EKF 在复杂场景下的稳定性。
