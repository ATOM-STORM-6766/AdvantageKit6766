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

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LoggedTunableBoolean;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private static final LoggedTunableBoolean ourIsFirstActive =
      new LoggedTunableBoolean("OurIsFirstActive", true);
  private static Timer matchTimer = new Timer();

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  @Override
  public void teleopPeriodic() {
    updateTeleopStatus();
  }

  @Override
  public void teleopExit() {
    matchTimer.stop();
    matchTimer.reset();
  }

  private void updateTeleopStatus() {
    boolean isOurShift = false;
    double time = matchTimer.get();
    double currentShiftTimeRemaining = 0.0;

    // Teleop is 140 seconds long (2:20 in the diagram)
    // Map time from 0 -> 140 to the shift logic
    // According to the image, the timeline shifts back and forth

    // If the timer is stopped or has been reset, we are not actively in teleop
    if (time == 0.0 && !matchTimer.isRunning()) {
      isOurShift = false;
      currentShiftTimeRemaining = 0.0;
    } else {
      // Logic for REEFSHIFT based on the provided image:
      // Teleop is from 2:20 -> 0:00 (140 seconds)
      // We will map elapsed time (0 to 140) to the remaining time (140 to 0) to match the chart.
      double remainingTime = 140.0 - time;

      // RED Alliance values are used as an example (assuming we check `ourIsFirstActive` as proxy
      // for Red/Blue logic or simply follow one path).
      // Here we will use the `ourIsFirstActive` boolean to decide whether we are the alliance that
      // is active during Shift 1.
      boolean startsActive = ourIsFirstActive.get();

      if (remainingTime > 130.0) {
        // TRANSITION SHIFT (2:20 - 2:10) -> 140s to 130s remaining
        isOurShift = true;
        currentShiftTimeRemaining = remainingTime - 130.0;
      } else if (remainingTime > 105.0) {
        // SHIFT 1 (2:10 - 1:45) -> 130s to 105s remaining
        isOurShift = startsActive;
        currentShiftTimeRemaining = remainingTime - 105.0;
      } else if (remainingTime > 80.0) {
        // SHIFT 2 (1:45 - 1:20) -> 105s to 80s remaining
        isOurShift = !startsActive;
        currentShiftTimeRemaining = remainingTime - 80.0;
      } else if (remainingTime > 55.0) {
        // SHIFT 3 (1:20 - 0:55) -> 80s to 55s remaining
        isOurShift = startsActive;
        currentShiftTimeRemaining = remainingTime - 55.0;
      } else if (remainingTime > 30.0) {
        // SHIFT 4 (0:55 - 0:30) -> 55s to 30s remaining
        isOurShift = !startsActive;
        currentShiftTimeRemaining = remainingTime - 30.0;
      } else if (remainingTime > 0.0) {
        // END GAME (0:30 - 0:00) -> 30s to 0s remaining
        // Both alliances are active
        isOurShift = true;
        currentShiftTimeRemaining = remainingTime; // remaining time itself is end game remaining
      } else {
        isOurShift = false;
        currentShiftTimeRemaining = 0.0;
      }

      // 闪烁逻辑：当在当前阶段剩余时间少于或等于 7 秒时，并且不处于 END GAME 阶段，让其快速切换状态
      if (currentShiftTimeRemaining <= 7.0 && currentShiftTimeRemaining > 0.0) {
        // 利用时间来使它实现 0.25秒亮、0.25秒灭，即2Hz的闪烁
        // time % 0.5 会在 0.0~0.5 之间波动，据此判断闪烁的占空比
        if ((time % 0.5) < 0.25) {
          isOurShift = !isOurShift;
        }
      }
    }

    Logger.recordOutput("Game/IsOurShiftActive", isOurShift);
    Logger.recordOutput("Game/CurrentShiftTimeRemaining", Math.max(0, currentShiftTimeRemaining));
    Logger.recordOutput("Game/TeleopTimeRemaining", Math.max(0, 140.0 - time));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // 可选：将当前线程切换到高优先级以改进循环计时（详见模板项目文档）
    // Threads.setCurrentThreadPriority(true, 99);

    // 运行调度器。负责：
    // 1. 轮询按钮
    // 2. 添加新调度的指令
    // 3. 运行已调度的指令
    // 4. 移除已完成或被中断的指令
    // 5. 调用各子系统的 periodic() 方法
    // 必须在 robotPeriodic() 中调用，Command 框架才能正常工作。
    CommandScheduler.getInstance().run();

    // 恢复为非实时线程优先级（不要修改第一个参数）
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.updateAutoChooser();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    Command resetCommand =
        new ParallelCommandGroup(
                robotContainer
                    .getHood()
                    .resetToLimitCommand()
                    .unless(robotContainer.getHood()::isInitialized),
                robotContainer
                    .getIntake()
                    .resetToLimitCommand()
                    .unless(robotContainer.getIntake()::isInitialized))
            .withName("Reset To Limit");

    CommandScheduler.getInstance().schedule(resetCommand);

    matchTimer.start();
    // CommandScheduler.getInstance().schedule(robotContainer.getTurret().resetToLimitCommand());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
