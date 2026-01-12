// Copyright 2021-2025 FRC 6766
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected = false;
    public double velocityRadPerSec = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Sets the target velocity for the flywheel in radians per second. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Sets the target velocity for the flywheel in RPM. */
  public default void setRPM(double rpm) {}

  /** Runs the flywheel motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}
}
