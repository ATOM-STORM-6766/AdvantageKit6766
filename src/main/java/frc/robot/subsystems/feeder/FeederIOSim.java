package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private static final DCMotor SHOOTER_MOTOR = DCMotor.getKrakenX60(1);
  private static final DCMotor INTAKE_MOTOR = DCMotor.getKrakenX60(1);

  private static final double SHOOTER_MOMENT_OF_INERTIA = 0.005;
  private static final double INTAKE_MOMENT_OF_INERTIA = 0.005;

  private final DCMotorSim shooterSim;
  private final DCMotorSim intakeSim;

  private final PIDController shooterController = new PIDController(0.1, 0.0, 0.0);
  private final PIDController intakeController = new PIDController(0.1, 0.0, 0.0);

  private boolean shooterClosedLoop = false;
  private boolean intakeClosedLoop = false;

  private double shooterAppliedVolts = 0.0;
  private double intakeAppliedVolts = 0.0;

  public FeederIOSim() {
    shooterSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(SHOOTER_MOTOR, SHOOTER_MOMENT_OF_INERTIA, 1.0),
            SHOOTER_MOTOR);
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_MOTOR, INTAKE_MOMENT_OF_INERTIA, 1.0),
            INTAKE_MOTOR);
  }

  @Override
  public void readInputs(FeederInputs inputs) {
    if (shooterClosedLoop) {
      shooterAppliedVolts = shooterController.calculate(shooterSim.getAngularVelocityRadPerSec());
      shooterAppliedVolts += shooterController.getSetpoint() * 0.1;
    }
    if (intakeClosedLoop) {
      intakeAppliedVolts = intakeController.calculate(intakeSim.getAngularVelocityRadPerSec());
      intakeAppliedVolts += intakeController.getSetpoint() * 0.1;
    }

    shooterSim.setInputVoltage(MathUtil.clamp(shooterAppliedVolts, -12.0, 12.0));
    intakeSim.setInputVoltage(MathUtil.clamp(intakeAppliedVolts, -12.0, 12.0));

    shooterSim.update(0.02);
    intakeSim.update(0.02);

    inputs.shooterVelocity = RPM.of(shooterSim.getAngularVelocityRPM());
    inputs.shooterAppliedVolts = Volts.of(shooterAppliedVolts);
    inputs.shooterTorqueCurrent = Amps.of(shooterSim.getCurrentDrawAmps());

    inputs.intakeVelocity = RPM.of(intakeSim.getAngularVelocityRPM());
    inputs.intakeAppliedVolts = Volts.of(intakeAppliedVolts);
    inputs.intakeTorqueCurrent = Amps.of(intakeSim.getCurrentDrawAmps());
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    shooterClosedLoop = true;
    shooterController.setSetpoint(velocity.in(RPM));
  }

  @Override
  public void setIntakeVelocity(AngularVelocity velocity) {
    intakeClosedLoop = true;
    intakeController.setSetpoint(velocity.in(RPM));
  }

  @Override
  public void stop() {
    shooterClosedLoop = false;
    intakeClosedLoop = false;
    shooterAppliedVolts = 0.0;
    intakeAppliedVolts = 0.0;
  }
}
