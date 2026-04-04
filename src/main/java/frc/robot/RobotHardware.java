package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.clamber.Clamber;
import frc.robot.subsystems.clamber.ClamberIO;
import frc.robot.subsystems.clamber.ClamberIOSim;
import frc.robot.subsystems.clamber.ClamberIOTalonFX;
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

public record RobotHardware(
    Drive drive,
    Vision vision,
    Flywheel flywheel,
    Feeder feeder,
    Hood hood,
    Intake intake,
    Clamber clamber) {
  public static RobotHardware create(Mode mode) {
    return switch (mode) {
      case REAL -> {
        Drive drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        Vision vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        yield new RobotHardware(
            drive,
            vision,
            new Flywheel(new FlywheelIOTalonFX(), new LimitSwitchDIO()),
            new Feeder(new FeederIOTalonFX()),
            new Hood(new HoodIOTalonFX()),
            new Intake(new IntakeIOTalonFX()),
            new Clamber(new ClamberIOTalonFX()));
      }
      case SIM -> {
        Drive drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        Vision vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        yield new RobotHardware(
            drive,
            vision,
            new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO()),
            new Feeder(new FeederIOSim()),
            new Hood(new HoodIOSim()),
            new Intake(new IntakeIOSim()),
            new Clamber(new ClamberIOSim()));
      }
      default -> {
        Drive drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        Vision vision =
            new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        yield new RobotHardware(
            drive,
            vision,
            new Flywheel(new FlywheelIOSim(), new LimitSwitchDIO()),
            new Feeder(new FeederIOSim()),
            new Hood(new HoodIOSim()),
            new Intake(new IntakeIOTalonFX()),
            new Clamber(new ClamberIO() {}));
      }
    };
  }
}
