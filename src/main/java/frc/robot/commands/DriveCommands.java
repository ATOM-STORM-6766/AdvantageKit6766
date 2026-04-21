// 版权 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// 本程序是自由软件；您可以根据自由软件基金会发布的 GNU 通用公共许可证
// 第 3 版的条款进行再发布和/或修改，或在本项目根目录中查阅该许可证。
//
// 本程序的发布目的是希望它有用，但不提供任何担保；
// 甚至没有适销性或特定用途适用性的默示担保。
// 详细信息请参阅 GNU 通用公共许可证。

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_START_DELAY = 2.0; // 秒
  private static final double FF_RAMP_RATE = 0.1; // 伏/秒
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // 弧度/秒
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // 弧度/秒^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // 应用死区
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // 将幅值平方以获得更精细的控制
    linearMagnitude = linearMagnitude * linearMagnitude;

    // 返回新的线速度向量
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  private static Rotation2d headRotation = new Rotation2d();

  /** 使用两个摇杆（分别控制线速度和角速度）的场相对驾驶指令。 */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    // Timer timer = new Timer();

    return
    // Commands.either(
    Commands.run(
            () -> {
              // 获取线速度
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              headRotation = drive.getRotation();
              // 对旋转输入应用死区
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // 将旋转输入平方以获得更精细的控制
              omega = Math.copySign(omega * omega, omega) * 0.5;

              // 转换为场相对速度并发送指令
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      AllianceFlipUtil.shouldFlip()
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        //     ,
        // joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> headRotation)
        //     .until(() -> MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) != 0.0),
        // () -> MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) == 0.0)
        .withName("Joystick Drive");
  }

  /** 使用两个摇杆（分别控制线速度和角速度）的机器人相对驾驶指令。 */
  public static Command joystickRobotDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    // Timer timer = new Timer();

    return Commands.run(
            () -> {
              // 获取线速度
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              headRotation = drive.getRotation();
              // 对旋转输入应用死区
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // 将旋转输入平方以获得更精细的控制
              omega = Math.copySign(omega * omega, omega) * 0.5;

              // 转换为场相对速度并发送指令
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega * drive.getMaxAngularSpeedRadPerSec());
              drive.runVelocity(speeds);
            },
            drive)
        .withName("Joystick Robot Drive");
  }

  /** 使用摇杆进行线速度控制、用 PID 进行角速度控制的场相对驾驶指令。 适用场景包括自动对准某个角度、瞄准视觉目标或通过摇杆控制绝对朝向。 */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    ProfiledPIDController angleController =
        Constants.DriveControlConstants.JoystickAngleHold.createAngleController();
    SimpleMotorFeedforward angleFeedforward =
        Constants.DriveControlConstants.JoystickAngleHold.createAngleFeedforward();

    // 构造指令
    return Commands.run(
            () -> {
              // 获取线速度
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // 计算角速度
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // 旋转角速度目标值 (弧度/秒)
              double omegaSetpoint = angleController.getSetpoint().velocity;

              double omegaFeedforward = angleFeedforward.calculate(omegaSetpoint);

              // 转换为场相对速度并发送指令
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega + omegaFeedforward);
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      AllianceFlipUtil.shouldFlip()
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // 指令开始时重置 PID 控制器
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  // 在限制范围内，将底盘角度旋转到30、120、210、300中最近的一个
  public static Command snapToNearest30Degrees(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> headRotation)
        .beforeStarting(
            () -> {
              Rotation2d currentAngle = drive.getRotation();
              double targetAngleDegrees =
                  Math.round((currentAngle.getDegrees() - 45.0) / 90.0) * 90.0 + 45.0;
              headRotation = Rotation2d.fromDegrees(targetAngleDegrees);
            })
        .withName("Snap To Nearest 30 Degrees");
  }

  /**
   * 测量驱动电机的速度前馈常数。
   *
   * <p>此指令仅应在电压控制模式下使用。
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // 重置数据
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // 允许各模块转向到位
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // 启动计时器
        Commands.runOnce(timer::restart),

        // 加速并采集数据
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // 指令结束时计算并输出结果
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** 通过原地旋转测量机器人的轮半径。 */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // 驱动控制序列
        Commands.sequence(
            // 重置加速度限制器
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // 原地旋转并加速到最大速度
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // 测量序列
        Commands.sequence(
            // 等待模块完全对齐后再开始测量
            Commands.waitSeconds(1.0),

            // 记录起始数据
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // 更新陀螺仪增量
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // 指令结束时计算并输出结果
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
