# 2026-04-06 Command 层重构设计

## 背景
本次重构目标是把当前自动阶段配置逻辑从单个 `AutoConfigurator` 中拆开，并同步把近期顶层硬件装配调整回收至 `RobotContainer`，使 command 层职责更清晰、更贴近 WPILib command-based 常见分层方式。

## 目标
1. 将 `AutoConfigurator` 拆成两部分：
   - 顶层自动模式选择与 `AutoFactory` 创建逻辑
   - 自动阶段 command/routine 逻辑
2. 将自动阶段 routine 逻辑迁移到 `src/main/java/frc/robot/commands/auto/` 下。
3. 结合当前 auto 与 `RobotControl` 中已有手柄触发 command，提取多个可复用原子 command，并按执行功能类别聚合到 `src/main/java/frc/robot/commands/` 下，减少单文件堆叠并统一入口。
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
- `frc.robot.commands.auto.routines.*`
  - 每个自动模式一个文件，例如 `P2Auto`、`P3Auto`、`P4Auto`、`P6Auto`、`P6MirrorAuto`
  - 每个文件只负责拼装该 routine 的 sequence/parallel 流程
- `frc.robot.commands.*` 下按执行功能聚合复用 command builder
  - 复用当前 `RobotControl` 中手柄绑定所使用的 command 组合
  - 供 teleop 与 auto 共用，避免 auto 再维护一套相似但割裂的原子逻辑
- 可选的辅助类（如需要）
  - 保持轻量，只承载共用的 command 组装，不制造多余层级

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

### 复用 command 聚合方式
计划把可复用原子 command 按执行功能归并到 `frc.robot.commands` 下，而不是只放在 `commands/auto`：
- **进球/喂料类**：封装 intake、wave、feed、stop feeding 等步骤
- **射球准备类**：封装固定点射球、自动瞄准射球、停止射球等步骤
- **定位/移动类**：封装 follow point、停底盘、必要的定点移动片段

拆分时会优先复用或上提当前 `RobotControl` 中已经直接使用的 command 组合，让 teleop 与 auto 使用同一套能力块。这样做有两个目的：
1. 避免自动阶段为了复用而再造一套只服务 auto 的原子 command；
2. 让 command 层按“执行意图”组织，而不是按“比赛阶段”组织。

落地形式优先选择少量聚合 builder，而不是把每个小动作都拆成独立 class。

### Routine 文件
每个 routine 文件暴露一个静态工厂方法，例如：
- `public static Command create(AutoFactory autoFactory, RobotContainer container, ... )`

每个文件内部只保留：
- routine 相关的关键 pose
- trajectory 获取与 sequence/parallel 组合
- 少量该 routine 独有的步骤
- 对 `frc.robot.commands` 下复用 command builder 的调用

这样可以把当前 `AutoConfigurator` 中长串的 `autoChooser.addCmd(...)` 展开为多个聚焦文件，同时让 auto 与手柄触发逻辑尽量复用同一套 command 能力块。

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
- 对于当前在 `RobotControl` 中直接内联的 command 组合，会优先评估是否上提到 `frc.robot.commands` 下按功能聚合的复用 builder；若已上提，则 `RobotControl` 直接调用这些 builder，而不是继续内联大段组合逻辑。
- 不做顺手功能修改。

## 注释与风格
- 新增注释统一使用中文。
- 仅在职责边界、分组和不够直观的拼装处添加注释，避免噪音。
- 保持 Spotless/google-java-format 可接受的结构。

## 迁移步骤
1. 新建 `AutoModeSelector`。
2. 将各自动模式拆分到 `commands/auto/routines/`。
3. 结合 auto 与 `RobotControl`，把可复用原子 command 上提到 `frc.robot.commands` 下按功能聚合。
4. 修改 `RobotContainer`，内联硬件构造并接入 selector。
5. 重排 `RobotControl`，改用新的复用 command builder 并补中文注释。
6. 运行构建检查，确保重构后行为保持一致。

## 风险与控制
- **风险：** 拆分后某些 auto routine 漏注册。
  - **控制：** 在 `AutoModeSelector` 中集中注册，保持与旧模式同名。
- **风险：** trajectory reset 时机在拆分中变化。
  - **控制：** 在 `AutoTrajectories` 中复刻现有 `.beforeStarting(resetOdometry(...))` 规则。
- **风险：** 复用 `RobotControl` 里的组合后，teleop 与 auto 共享 command builder 时不小心带入阶段特有假设。
  - **控制：** builder 只抽“动作能力”，不抽按钮语义；auto 特有的时序仍保留在 routine 文件中。
- **风险：** `RobotContainer` 回收硬件装配时引入构造差异。  - **控制：** 以 `origin/event-ShangHai` 对应实现为基线，只做当前分支必要的最小回收。
- **风险：** `RobotControl` 重排时误改绑定行为。
  - **控制：** 仅移动代码位置与补注释，不改按钮到 command 的映射关系。

## 验收标准
- `AutoConfigurator` 不再承担当前 chooser + routines 的混合职责。
- `AutoModeSelector` 位于 `frc.robot`，并被 `RobotContainer` 使用。
- 自动 routine 拆分到 `commands/auto/routines/`，同时复用原子 command 按功能聚合在 `frc.robot.commands` 下，而不是只服务 auto。
- `RobotHardware` 不再单独成文件，硬件实例化逻辑回到 `RobotContainer`。
- `RobotControl` 行为不变，但 controller/operator 逻辑分组更清晰，且注释为中文。
- 项目可通过 `./gradlew build` 基本编译验证。
