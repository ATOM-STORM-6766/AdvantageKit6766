# 2026-04-06 Command 层重构设计

## 背景
本次重构目标是把当前自动阶段配置逻辑从单个 `AutoConfigurator` 中拆开，并同步把近期顶层硬件装配调整回收至 `RobotContainer`，使 command 层职责更清晰、更贴近 WPILib command-based 常见分层方式。

## 目标
1. 将 `AutoConfigurator` 拆成两部分：
   - 顶层自动模式选择与 `AutoFactory` 创建逻辑
   - 自动阶段 command/routine 逻辑
2. 将自动阶段 command 逻辑迁移到 `src/main/java/frc/robot/commands/auto/` 下。
3. 在新的 `commands/auto/` 下，提取多个可复用原子 command，减少单文件堆叠。
4. 新增 `src/main/java/frc/robot/AutoModeSelector.java`，由 `RobotContainer` 使用。
5. 取消独立 `RobotHardware` 文件，参考 `event-ShangHai` 的装配方式，将硬件实例化收回 `RobotContainer.java`。
6. 保持 `RobotControl` 功能不变，仅重排 controller/operator 逻辑并补充中文注释。

## 非目标
- 不改变现有自动模式名称、自动路径名称、按钮功能或比赛逻辑。
- 不顺手重写 Aim/Pass/Drive 等既有 command 的行为。
- 不额外抽取与本次重构无关的常量层或配置层。

## 目标结构

### 顶层
- `frc.robot.AutoModeSelector`
  - 持有 `AutoChooser`
  - 负责在 alliance 可用时创建 `AutoFactory`
  - 负责注册所有 auto routines
  - 对 `RobotContainer` 暴露最小接口：更新 chooser、获取自动指令

### command 层
- `frc.robot.commands.auto.AutoCommands`
  - 提供自动阶段可复用的原子 command builder
  - 封装 reset/intake/feed/shoot/follow point/stop drive 等重复段落
- `frc.robot.commands.auto.AutoTrajectories`
  - 封装 `AutoFactory` 的轨迹命令与 resetOdometry 组合
  - 统一 trajectory 命名，避免在 routine 文件里重复样板代码
- `frc.robot.commands.auto.routines.*`
  - 每个自动模式一个文件，例如 `P2Auto`、`P3Auto`、`P4Auto`、`P6Auto`、`P6MirrorAuto`
  - 每个文件只负责拼装该 routine 的 sequence/parallel 流程

## 详细职责

### AutoModeSelector
- 构造时接收 `RobotContainer` 与 auto 相关子系统依赖。
- 内部维护：
  - `AutoChooser`
  - `AutoFactory`（延迟初始化）
- `update()`：
  - 当 alliance 已可用且 factory 尚未创建时，创建 `AutoFactory`
  - 调用 routines 注册逻辑
  - 更新 dashboard/logger
- `getSelectedCommand()`：返回当前选中的自动指令。
- `getChooser()`：仅在 `RobotContainer` 需要向 SmartDashboard 注册时使用；若实现中直接由 selector 注册，则可不暴露。

### AutoCommands
计划抽取以下原子 command：
- `resetMechanisms()`：重置 intake 与 hood
- `runIntake()`：自动阶段进球动作
- `feedWithWave(double delaySeconds)`：喂料 + intake wave
- `autoAimShoot(double timeoutSeconds)`：自动瞄准并出球
- `stopShooting()`：停止 flywheel、feeder，并收回 intake
- `shootAtFixedPosition()`：固定 hood/flywheel 参数出球
- `followPoint(Pose2d pose)`：包装 `FollowPoint` 并处理 alliance flip
- `stopDrive()` / `zeroChassisSpeeds()`：统一底盘停转动作

这些原子 command 不追求“每行一个类”的极端拆分，而是集中在一个 builder 类里，保持复用与可读性的平衡。

### AutoTrajectories
计划提供：
- `trajectory(String name)`
- `trajectory(String name, int splitIndex)`
- 自动带上 `.beforeStarting(resetOdometry(...))` 的首段包装
- 命名规范统一，例如 `p6Part1()` 这类具名方法或最小成本的通用方法调用

这里优先选择“轻包装”，避免为每条路径再造额外抽象。

### Routine 文件
每个 routine 文件暴露一个静态工厂方法，例如：
- `public static Command create(AutoTrajectories trajectories, AutoCommands commands)`

每个文件内部只保留：
- routine 相关的关键 pose
- sequence/parallel 组合
- 少量该 routine 独有的步骤

这样可以把当前 `AutoConfigurator` 中长串的 `autoChooser.addCmd(...)` 展开为多个聚焦文件。

## RobotContainer 调整
- 删除对独立 `RobotHardware` 文件的依赖。
- 按 `event-ShangHai` 的模式，在 `RobotContainer` 内直接按 `REAL/SIM/REPLAY` 构造各 subsystem。
- 保留当前已有字段与 getter，避免影响外部 command。
- 用 `AutoModeSelector` 替换当前的 `AutoChooser + AutoFactory + AutoConfigurator` 组合。
- `updateAutoChooser()` 改为委托给 selector。
- `getAutonomousCommand()` 改为从 selector 读取。

## RobotControl 调整
- 不改变任何按键行为和 command 组合。
- 仅重排结构，使以下内容各自聚合：
  - 默认驾驶 / controller 驾驶相关绑定
  - operator 相关绑定
- 添加中文分区注释，帮助快速定位。
- 不做顺手功能修改。

## 注释与风格
- 新增注释统一使用中文。
- 仅在职责边界、分组和不够直观的拼装处添加注释，避免噪音。
- 保持 Spotless/google-java-format 可接受的结构。

## 迁移步骤
1. 新建 `AutoModeSelector`。
2. 新建 `commands/auto/AutoCommands` 与 `AutoTrajectories`。
3. 将各自动模式拆分到 `commands/auto/routines/`。
4. 修改 `RobotContainer`，内联硬件构造并接入 selector。
5. 重排 `RobotControl` 并补中文注释。
6. 运行构建检查，确保重构后行为保持一致。

## 风险与控制
- **风险：** 拆分后某些 auto routine 漏注册。
  - **控制：** 在 `AutoModeSelector` 中集中注册，保持与旧模式同名。
- **风险：** trajectory reset 时机在拆分中变化。
  - **控制：** 在 `AutoTrajectories` 中复刻现有 `.beforeStarting(resetOdometry(...))` 规则。
- **风险：** `RobotContainer` 回收硬件装配时引入构造差异。
  - **控制：** 以 `origin/event-ShangHai` 对应实现为基线，只做当前分支必要的最小回收。
- **风险：** `RobotControl` 重排时误改绑定行为。
  - **控制：** 仅移动代码位置与补注释，不改按钮到 command 的映射关系。

## 验收标准
- `AutoConfigurator` 不再承担当前 chooser + routines 的混合职责。
- `AutoModeSelector` 位于 `frc.robot`，并被 `RobotContainer` 使用。
- 自动 command 逻辑迁移到 `commands/auto/`，并包含可复用原子 command。
- `RobotHardware` 不再单独成文件，硬件实例化逻辑回到 `RobotContainer`。
- `RobotControl` 行为不变，但 controller/operator 逻辑分组更清晰，且注释为中文。
- 项目可通过 `./gradlew build` 基本编译验证。
