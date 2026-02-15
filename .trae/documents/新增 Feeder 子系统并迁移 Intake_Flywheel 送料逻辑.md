## 目标
- 按 Hood 子系统的目录与分层风格新增 Feeder 子系统
- 将 Intake 与 Flywheel 内部的送料电机逻辑完整迁移到 Feeder

## 现状要点（基于代码检索）
- Hood 结构示例：Hood + HoodIO + HoodIOTalonFX + HoodIOSim + HoodConstants
- Flywheel 内含射球口 Feeder（shootFeedMotor、setFeederVelocity、Feeder TalonFX config）
- Intake 内含 Feed Motor（feedMotorID=22、setFeedVelocity）
- RobotContainer 中已有对 Flywheel/Intake 送料命令的绑定

## 实施方案
- 新建 `frc.robot.subsystems.feeder` 目录，提供：
  - Feeder.java、FeederIO.java、FeederIOTalonFX.java、FeederIOSim.java、FeederConstants.java
  - 结构、日志、Command 风格与 Hood 保持一致
- 从 Flywheel 迁移：
  - Feeder IO 字段、输入采集、setFeederVelocity 逻辑
  - shootFeedMotor 的 TalonFX 初始化与刷新信号
  - Feeder TalonFX 配置常量
  - Flywheel 仅保留 3 个飞轮电机控制
- 从 Intake 迁移：
  - feedMotor 与 feedVelocity、setFeedVelocity
  - feedMotorID 与相关常量
  - Intake 只保留进球电机与翻爪等非送料逻辑
- 调整 RobotContainer：
  - 所有与送料相关的绑定改用 Feeder 子系统命令
  - 旧的 Intake/Flywheel 送料调用移除或替换

## 验证方式
- 通过编译或相关单元测试验证构建通过
- 重点检查日志与输入采集路径是否与其它子系统一致

如果确认这套迁移边界（Intake + Flywheel 的送料都归 Feeder），我将按此计划开始修改。
