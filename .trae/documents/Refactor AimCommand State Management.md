## 目标与结论
1. 把自瞄解算从 AimCommand 的静态状态迁移到 AimSubsystem 的实例状态，并在其 periodic 中更新 RobotState。
2. AimSubsystem 统一产出 ShooterSetpoint，并提供 flywheel/hood 需要的 setpoint 访问器。
3. 新增一个键位（默认使用 Y）实现“按下预热 → 达标后喂球 → 松手停喂球与飞轮”。
4. GenericShooterResolver 不做强制改动；若后续要更高可测性再引入依赖注入。

## 具体改造点
1. 新建 AimSubsystem
   - 字段：targetSupplier、lastValidSetpoint。
   - periodic：从 RobotState 取姿态/速度 + targetSupplier 解算，更新 lastValidSetpoint，并写入 RobotState（新增接口）。
   - 提供 getFlywheelSetpoint()、getHoodPitchRad()、isSetpointValid() 等访问器。
2. 扩展 RobotState
   - 新增存储/读取 ShooterSetpoint 的字段与方法（例如 setAimSetpoint / getAimSetpoint）。
3. 调整 AimCommand 或新增命令工厂
   - 去除 static 状态，改为使用 AimSubsystem 的访问器。
   - 旧的 LB 自瞄逻辑保持，只是改为设置 AimSubsystem 的 targetSupplier。
4. 扩展 Flywheel/Feeder 命令能力
   - Flywheel：新增“持续更新 setpoint”的 command（run），用于自瞄动态变化。
   - Feeder：新增“持续输出速度”的 command（run），避免 runOnce 导致序列提前结束。
5. RobotContainer 按键
   - 新增 Y 键绑定：并行执行
     - 预热：flywheel+hood 持续跟随 AimSubsystem setpoint
     - 等待：flywheel.waitForVelocity + hood.waitForPosition（且 setpoint 有效）
     - 达标后：启动两个 feeder 的持续输出
   - 松手 onFalse：停止 feeder 并停止飞轮（符合你的新要求）。

## 验证方式
1. 运行现有模拟/机器人模式日志，确认 AimSubsystem periodic 输出与 RobotState 中 setpoint 更新。
2. 按住新键位时：飞轮/hood 达标前 feeder 不启动，达标后 feeder 启动；松手 feeder 与飞轮立即停止。
3. LB 自瞄仍可正常工作，且 setpoint 由 AimSubsystem 统一管理。
