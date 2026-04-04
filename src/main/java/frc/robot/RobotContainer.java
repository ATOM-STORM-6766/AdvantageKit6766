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

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.match.AutoConfigurator;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.aim.PassSubsystem;
import frc.robot.subsystems.clamber.Clamber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * 该类用于声明机器人中的主要内容。由于 Command-based 属于“声明式”范式， {@link Robot} 的 periodic 方法中（除调度器调用外）不应包含太多机器人逻辑。
 * 相反，机器人结构（子系统、指令和按键映射等）应在此处声明。
 */
public class RobotContainer {
  // 子系统
  private final Drive drive;
  private final Clamber clamber;

  private AutoFactory autoFactory;
  private final AutoChooser autoChooser;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Flywheel flywheel;
  private final Feeder feeder;

  private final Hood hood;
  private final Intake intake;
  private final AimSubsystem aimSubsystem = new AimSubsystem();
  private final PassSubsystem passSubsystem = new PassSubsystem();

  // 控制器
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(1);

  /** 机器人容器，包含子系统、操作接口设备以及指令。 */
  public RobotContainer() {
    RobotHardware hardware = RobotHardware.create(Constants.currentMode);
    drive = hardware.drive();
    vision = hardware.vision();
    flywheel = hardware.flywheel();
    feeder = hardware.feeder();
    hood = hardware.hood();
    intake = hardware.intake();
    clamber = hardware.clamber();

    // 设置自动例程
    autoChooser = new AutoChooser();
    autoFactory = null;
    SmartDashboard.putData("Auto Choices", autoChooser);
    updateAutoChooser();

    // 配置按键绑定
    configureButtonBindings();
  }

  public void updateAutoChooser() {
    if (autoFactory == null) {
      if (DriverStation.getAlliance().isPresent()) {
        autoFactory =
            new AutoConfigurator(this, drive, flywheel, feeder, hood, intake)
                .configure(autoChooser);
        SmartDashboard.putData("Auto Choices", autoChooser);
        Logger.recordOutput("Robot/Auto/ChooserReady", true);
      } else {
        Logger.recordOutput("Robot/Auto/ChooserReady", false);
      }
    }
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

  public Clamber getClamber() {
    return clamber;
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

  /**
   * 使用此方法定义按键到指令的映射。可以实例化 {@link GenericHID} 或其子类 （如 {@link edu.wpi.first.wpilibj.Joystick} 或
   * {@link XboxController}），然后传递给 {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}
   * 来创建按钮。
   */
  private void configureButtonBindings() {
    new RobotControl(this, drive, clamber, flywheel, feeder, intake, controller, operator)
        .configure();
  }

  /**
   * 用此方法将自动阶段的指令传递给 {@link Robot} 主类。
   *
   * @return 自动阶段需要运行的指令
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
