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

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowChoreoPathCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.LimitSwitchDIO;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * 该类用于声明机器人中的主要内容。由于 Command-based 属于“声明式”范式， {@link Robot} 的 periodic 方法中（除调度器调用外）不应包含太多机器人逻辑。
 * 相反，机器人结构（子系统、指令和按键映射等）应在此处声明。
 */
public class RobotContainer {
  // 子系统
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Flywheel flywheel;
  private final Feeder feeder;

  private final Hood hood;
  private final Intake m_intake;
  private final AimSubsystem aimSubsystem = new AimSubsystem();

  // 控制器
  private final CommandXboxController controller = new CommandXboxController(0);

  // 仪表板输入
  private final LoggedDashboardChooser<Command> autoChooser;

  /** 机器人容器，包含子系统、操作接口设备以及指令。 */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // 真机模式：实例化硬件 IO 实现
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

        hood = new Hood(new HoodIOTalonFX());
        flywheel = new Flywheel(new FlywheelIOTalonFX(), new LimitSwitchDIO(0, 1, 2));
        feeder = new Feeder(new FeederIOTalonFX());
        m_intake = new Intake(new IntakeIOTalonFX());
        break;

      case SIM:
        // 仿真模式：实例化物理仿真 IO 实现
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        hood = new Hood(new HoodIOSim());
        flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO(0, 1, 2));
        feeder = new Feeder(new FeederIOSim());
        m_intake = new Intake(new IntakeIOSim());
        break;

      default:
        // 回放模式：禁用 IO 实现
        // （使用与真机相同数量的虚拟实现）
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        hood = new Hood(new HoodIOSim());
        flywheel = new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO(0, 1, 2));
        feeder = new Feeder(new FeederIOSim());
        m_intake = new Intake(new IntakeIOTalonFX());
        break;
    }

    // 设置自动例程
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // 设置 SysId 例程
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Intake SysId", m_intake.runSysId());
    autoChooser.addOption(
        "test", new FollowChoreoPathCommand(drive, Choreo.loadTrajectory("test")));

    // 配置按键绑定
    configureButtonBindings();
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
    return m_intake;
  }

  public AimSubsystem getAimSubsystem() {
    return aimSubsystem;
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
    // 默认指令：正常的场相对驾驶
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // 按下 X 键时切换至 X 模式
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // 按住右肩键时启动 intake 吸球
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                m_intake.setIntakeVelocityCommand(Volts.of(8.0)),
                m_intake.setPosCommand(Rotation.of(0))))
        .onFalse(Commands.parallel(m_intake.stopCommand(), feeder.stopCommand()));

    // 按住左肩键时自瞄目标
    controller
        .leftBumper()
        .whileTrue(
            AimCommand.autoAimAtTarget(
                this,
                () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()))
        .onFalse(Commands.parallel(flywheel.stopCommand()));

    // 按下 y 键时送球
    controller
        .y()
        .whileTrue(
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(60), () -> RotationsPerSecond.of(20)))
        .onFalse(Commands.parallel(feeder.stopCommand()));
  }

  /**
   * 用此方法将自动阶段的指令传递给 {@link Robot} 主类。
   *
   * @return 自动阶段需要运行的指令
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
