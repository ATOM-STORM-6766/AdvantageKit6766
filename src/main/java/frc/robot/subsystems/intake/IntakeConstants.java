package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
  public static final CANBus canBus = CANBus.roboRIO();

  public static final int intakeMotorID = 41;
  public static final int feedMotorID = 2;
  public static final int positionMotorID = 50; // todo: change me
}
