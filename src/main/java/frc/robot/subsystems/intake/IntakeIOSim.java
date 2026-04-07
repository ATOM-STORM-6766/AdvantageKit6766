package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor POSITION_MOTOR = DCMotor.getKrakenX60(1);
  private static final DCMotor INTAKE_MOTOR = DCMotor.getKrakenX44(1);

  // 这里的惯性需要根据实际机械结构估算
  private static final double POSITION_MOMENT_OF_INERTIA = 0.05;
  private static final double INTAKE_MOMENT_OF_INERTIA = 0.005;

  private final DCMotorSim positionSim;
  private final DCMotorSim intakeSim;

  // 简单的 PID 控制器模拟闭环控制
  private final PIDController positionController = new PIDController(10.0, 0.0, 0.0);

  private boolean positionClosedLoop = false;

  private double positionClosedLoopOutputLimitVolts = 12.0;
  private double positionAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

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

    positionSim.setState(Units.rotationsToRadians(IntakeConstants.minMechanismRotations), 0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // 运行闭环控制计算
    if (positionClosedLoop) {
      positionAppliedVolts = positionController.calculate(positionSim.getAngularPositionRad());
    }
    positionSim.setInputVoltage(
        MathUtil.clamp(
            positionAppliedVolts,
            -positionClosedLoopOutputLimitVolts,
            positionClosedLoopOutputLimitVolts));
    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));

    positionSim.update(0.02);
    intakeSim.update(0.02);

    double posRad = positionSim.getAngularPositionRad();
    double minRad = Units.rotationsToRadians(IntakeConstants.minMechanismRotations);
    double maxRad = Units.rotationsToRadians(IntakeConstants.maxMechanismRotations);

    if (posRad < minRad) {
      positionSim.setState(minRad, 0);
    } else if (posRad > maxRad) {
      positionSim.setState(maxRad, 0);
    }

    inputs.intakePosition = positionSim.getAngularPosition();
    inputs.positionVelocity = positionSim.getAngularVelocity();
    inputs.positionCurrent = Amp.of(positionSim.getCurrentDrawAmps());
    inputs.intakeVelocity = intakeSim.getAngularVelocity();
  }

  @Override
  public void setIntakePosition(Distance position) {
    positionClosedLoop = true;
    positionClosedLoopOutputLimitVolts = 12.0;
    positionController.setSetpoint(
        Rotations.of(position.in(Meters) / IntakeConstants.positionMetersPerMechanismRotation)
            .in(Radians));
  }

  @Override
  public void setIntakeVelocity(Voltage voltage) {
    intakeAppliedVolts = voltage.in(Volts);
  }

  @Override
  public void setPositionVoltage(Voltage voltage) {
    positionClosedLoop = false;
    positionClosedLoopOutputLimitVolts = 12.0;
    positionAppliedVolts = voltage.in(Volts);
  }

  @Override
  public void setPosition(double positionRotations) {
    positionSim.setState(
        Units.rotationsToRadians(positionRotations), positionSim.getAngularVelocityRadPerSec());
  }

  @Override
  public void setPositionCurrent(Current current) {
    // 这里我们简单地将电流转换为电压，实际情况可能需要更复杂的模型
    double voltage = current.in(Amps) * 0.5; // 假设每安培对应0.5伏特
    setPositionVoltage(Volts.of(voltage));
  }
}
