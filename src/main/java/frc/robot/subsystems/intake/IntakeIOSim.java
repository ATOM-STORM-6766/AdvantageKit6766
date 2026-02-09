package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor POSITION_MOTOR = DCMotor.getKrakenX60(1);
  private static final DCMotor INTAKE_MOTOR = DCMotor.getKrakenX44(1);
  private static final DCMotor FEED_MOTOR = DCMotor.getKrakenX60(1);

  // 这里的惯性需要根据实际机械结构估算
  private static final double POSITION_MOMENT_OF_INERTIA = 0.05;
  private static final double INTAKE_MOMENT_OF_INERTIA = 0.005;
  private static final double FEED_MOMENT_OF_INERTIA = 0.005;

  private final DCMotorSim positionSim;
  private final DCMotorSim intakeSim;
  private final DCMotorSim feedSim;

  // 简单的 PID 控制器模拟闭环控制
  private final PIDController positionController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController feedController = new PIDController(0.1, 0.0, 0.0);

  private boolean positionClosedLoop = false;
  private boolean feedClosedLoop = false;

  private double positionAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;
  private double feedAppliedVolts = 0.0;

  public IntakeIOSim() {
    // 初始化 Intake 展开机构的仿真模型
    positionSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                POSITION_MOTOR, POSITION_MOMENT_OF_INERTIA, IntakeConstants.positionGearRatio),
            POSITION_MOTOR);

    // 初始化滚轮电机的仿真模型
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_MOTOR, INTAKE_MOMENT_OF_INERTIA, 1.0),
            INTAKE_MOTOR);

    feedSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(FEED_MOTOR, FEED_MOMENT_OF_INERTIA, 1.0),
            FEED_MOTOR);

    // 初始位置设置，例如收起状态
    positionSim.setState(Units.rotationsToRadians(IntakeConstants.minRotation), 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // 运行闭环控制计算
    if (positionClosedLoop) {
      positionAppliedVolts = positionController.calculate(positionSim.getAngularPositionRad());
    }
    if (feedClosedLoop) {
      // 这里的单位需要注意，DCMotorSim 输出是 rad/s
      feedAppliedVolts = feedController.calculate(feedSim.getAngularVelocityRadPerSec());
      // Feed forward 简化处理
      feedAppliedVolts += feedController.getSetpoint() * 0.1; // 简单的 kF
    }

    // 应用电压并更新物理仿真
    positionSim.setInputVoltage(MathUtil.clamp(positionAppliedVolts, -12.0, 12.0));
    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));
    feedSim.setInputVoltage(MathUtil.clamp(feedAppliedVolts, -12.0, 12.0));

    positionSim.update(0.02);
    intakeSim.update(0.02);
    feedSim.update(0.02);

    // 可以在这里添加硬限位的逻辑
    double posRad = positionSim.getAngularPositionRad();
    double minRad = Units.rotationsToRadians(IntakeConstants.minRotation);
    double maxRad = Units.rotationsToRadians(IntakeConstants.maxRotation);

    if (posRad < minRad) {
      positionSim.setState(minRad, 0);
    } else if (posRad > maxRad) {
      positionSim.setState(maxRad, 0);
    }

    // 更新 Inputs
    inputs.intakePosition = positionSim.getAngularPosition();
    inputs.positionVelocity = positionSim.getAngularVelocity();
    inputs.positionCurrent = Amp.of(positionSim.getCurrentDrawAmps());
    inputs.intakeVelocity = intakeSim.getAngularVelocity();
    inputs.feedVelocity = feedSim.getAngularVelocity();
  }

  @Override
  public void setIntakePosition(Angle position) {
    positionClosedLoop = true;
    positionController.setSetpoint(position.in(Radians));
  }

  @Override
  public void setIntakeVelocity(Voltage voltage) {
    intakeAppliedVolts = voltage.in(Volts);
  }

  @Override
  public void setFeedVelocity(AngularVelocity velocity) {
    feedClosedLoop = true;
    feedController.setSetpoint(velocity.in(RadiansPerSecond));
  }

  @Override
  public void setPositionVoltage(Voltage voltage) {
    positionClosedLoop = false;
    positionAppliedVolts = voltage.in(Volts);
  }

  @Override
  public void setCurrentPosition(double positionRotations) {
    positionSim.setState(
        Units.rotationsToRadians(positionRotations), positionSim.getAngularVelocityRadPerSec());
  }
}
