// Copyright 2021-2023 FRC 6328
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

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  boolean autonInitCommandRun = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  
    m_robotContainer = new RobotContainer();

    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    SignalLogger.enableAutoLogging(false);

    LED.setState(LEDState.RED);


    // Command disabledCommand = new PathPlannerAuto("Dummy-Auto-Fix-Auto").ignoringDisable(true);
    // disabledCommand.schedule();
    // Timer.delay(1);
    // disabledCommand.cancel();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = new WaitCommand(0.01).andThen(m_robotContainer.getAutonomousCommand());

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {

      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (RobotContainer.m_drivebase.getIsSetGyroRequestPresent()) {
      var alliance = DriverStation.getAlliance();

      if (RobotContainer.m_drivebase.getIsGyroRequestAmpSide()) {
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          RobotContainer.m_drivebase.addGyro(-60);
        } else {
          RobotContainer.m_drivebase.addGyro(60);
        }
      } else {
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          RobotContainer.m_drivebase.addGyro(60);
        } else {
          RobotContainer.m_drivebase.addGyro(-60);
        }
      }
      RobotContainer.m_drivebase.setGyroRequest(false, false);
    }

    RobotContainer.m_drivebase.resetPoseEstimator(
    new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(RobotContainer.m_drivebase.getYawDegrees()),
      RobotContainer.m_drivebase.getModulePositions(),
      RobotContainer.m_drivebase.getPose(),
      VecBuilder.fill(0.1, 0.1, 0.0),
      VecBuilder.fill(0.9, 0.9, 100.0))
    );
    // RobotContainer.m_drivebase.updateEstimatorWithPose(RobotContainer.m_drivebase.getPose());

    LED.setState(LEDState.TEAL_STOW);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    LED.setState(LEDState.RED);

    if (autonInitCommandRun == false) {
      Command autonInitCommand = FollowPathCommand.warmupCommand();
      // new PathPlannerAuto("1 Meter Auto").ignoringDisable(true);
      autonInitCommand.schedule();
      autonInitCommandRun = true;
    }

  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
