// 版权 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// 本程序是自由软件；您可以根据自由软件基金会发布的 GNU 通用公共许可证
// 第 3 版的条款进行再发布和/或修改，或在本项目根目录中查阅该许可证。
//
// 本程序的发布目的是希望它有用，但不提供任何担保；
// 甚至没有适销性或特定用途适用性的默示担保。
// 详细信息请参阅 GNU 通用公共许可证。

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControlConstants.FollowPoint;
import frc.robot.Constants.DriveControlConstants.JoystickAngleHold;
import frc.robot.Constants.DriveControlConstants.TrajectoryFollow;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.aim.PassSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.light.Light;
import frc.robot.subsystems.vision.Vision;

/**
 * 该类用于声明机器人中的主要内容。由于 Command-based 属于“声明式”范式， {@link Robot} 的 periodic 方法中（除调度器调用外）不应包含太多机器人逻辑。
 * 相反，机器人结构（子系统、指令和按键映射等）应在此处声明。
 */
public class RobotContainer {
  // 子系统
  private final Drive drive;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;
  private final AimSubsystem aimSubsystem;
  private final PassSubsystem passSubsystem;
  private final Light light;

  // 控制器
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);

  // 指令注册
  private final AutoModeSelector autoModeSelector;
  private final RobotControl robotControl;

  /** 机器人容器，包含子系统、操作接口设备以及指令。 */
  public RobotContainer() {
    aimSubsystem = new AimSubsystem();
    passSubsystem = new PassSubsystem();
    light = new Light();
    drive = new Drive();
    vision = new Vision(drive::addVisionMeasurement, drive::getPose);
    flywheel = new Flywheel();
    feeder = new Feeder();
    hood = new Hood();
    intake = new Intake();

    autoModeSelector = new AutoModeSelector(this);
    autoModeSelector.update();
    robotControl = new RobotControl(this);
    robotControl.configure();

    TrajectoryFollow.publishToSmartDashboard();
    FollowPoint.publishToSmartDashboard();
    JoystickAngleHold.publishToSmartDashboard();
  }

  public Hood getHood() {
    return hood;
  }

  public Flywheel getFlywheel() {
    return flywheel;
  }

  public Feeder getFeeder() {
    return feeder;
  }

  public Intake getIntake() {
    return intake;
  }

  public AimSubsystem getAimSubsystem() {
    return aimSubsystem;
  }

  public PassSubsystem getPassSubsystem() {
    return passSubsystem;
  }

  public Drive getDrive() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }

  public Light getLight() {
    return light;
  }

  public AutoModeSelector getAutoModeSelector() {
    return autoModeSelector;
  }

  public CommandXboxController getController() {
    return controller;
  }

  public CommandPS5Controller getOperator() {
    return operator;
  }
}
