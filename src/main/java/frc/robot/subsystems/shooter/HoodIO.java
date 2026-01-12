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

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean atForwardLimit = false;
    public boolean atReverseLimit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Sets the target position for the hood. */
  public default void setPosition(Rotation2d angle) {}

  /** Runs the hood motor at the specified open loop value. */
  public default void setOpenLoop(double output) {}

  /** Gets the current position of the hood. */
  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  /** Resets the hood to the limit position using stall current detection. */
  public default void resetToLimit() {}
}
