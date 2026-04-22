// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  static VisionIO[] createAll(
      java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
    switch (frc.robot.Constants.currentMode) {
      case REAL:
        return new VisionIO[] {
          new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
          new VisionIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1),
          new VisionIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2)
        };
      case SIM:
        return new VisionIO[] {
          new VisionIOPhotonVisionSim(
              VisionConstants.camera0Name, VisionConstants.robotToCamera0, poseSupplier),
          new VisionIOPhotonVisionSim(
              VisionConstants.camera1Name, VisionConstants.robotToCamera1, poseSupplier),
          new VisionIOPhotonVisionSim(
              VisionConstants.camera2Name, VisionConstants.robotToCamera2, poseSupplier)
        };
      default:
        return new VisionIO[] {new VisionIO() {}, new VisionIO() {}, new VisionIO() {}};
    }
  }
}
