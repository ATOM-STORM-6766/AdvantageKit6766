package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static AprilTagFieldLayout aprilTagLayout = // TODO: 更新为正确的布局
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();
  public static final double fieldLength = aprilTagLayout.getFieldLength();

  public static class Hub {

    // Dimensions
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // Relevant reference points on alliance blue side
    public static final Translation3d topCenterPoint =
        new Translation3d(
            aprilTagLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0, height);

    public static final Translation2d passPoint =
        new Translation2d(2.7938103675842285, 5.905640125274658);

    public static final Pose2d tower =
        new Pose2d(1.58, aprilTagLayout.getTagPose(31).get().getY(), Rotation2d.k180deg);

    public static final Pose2d blink =
        new Pose2d(
            2.792485475540161,
            1.834878921508789,
            Rotation2d.fromRadians(0.8832554013068488).rotateBy(Rotation2d.kPi));
  }

  public static AprilTagFieldLayout getAprilTagLayout() {
    return aprilTagLayout;
  }
}
