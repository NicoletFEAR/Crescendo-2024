// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utilities.Alert;
import frc.lib.utilities.LoggedDashboardChooser;
import frc.lib.utilities.Alert.AlertType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.CenterNoteAuto;
import frc.robot.commands.drivebase.TeleopSwerve;
import frc.robot.commands.drivebase.TurnToAngle;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.launcher.LauncherSuperstructure;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;
import frc.robot.subsystems.swerve.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // SWERVE CONTROLS \\
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // CONTROLLERS \\
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  public static ShuffleboardTab infoTab = kInfoMode ? Shuffleboard.getTab("Info") : null;
  public static ShuffleboardTab driveTuningTab =
      kTuningMode ? Shuffleboard.getTab("Drive Tuning") : null;
  public static ShuffleboardTab mechTuningTab =
      kTuningMode ? Shuffleboard.getTab("Mech Tuning") : null;

  // SUBSYSTEMS \\
  private SwerveDrive m_drivebase = SwerveDrive.getInstance();
  public static LED m_superStructureLED = new LED(new AddressableLED(0), 20);
  private LauncherSuperstructure m_launcherSuperstructure = LauncherSuperstructure.getInstance();

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser;

  // ALERTS \\
  private Alert infoAlert =
      new Alert("Info Mode Activated, expect decreased network performance.", AlertType.INFO);
  private Alert tuningAlert =
      new Alert("Tuning Mode Activated, expect decreased network performance.", AlertType.INFO);

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\
    // you would never do this while following a path, its just to show how to implement

    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Picker", AutoBuilder.buildAutoChooser(), mainTab, 0, 0, 2, 1);
    autoChooser.addOption("Center Command", new SequentialCommandGroup(new InstantCommand(() -> m_drivebase.updateEstimatorWithPose(new Pose2d(2, 0.76, Rotation2d.fromDegrees(0)))), 
    new CenterNoteAuto()));

    // CONFIGURE DEFAULT COMMANDS \\
    m_drivebase.setDefaultCommand(
        new TeleopSwerve(
            m_drivebase,
            m_driverController,
            translationAxis,
            strafeAxis,
            rotationAxis,
            true,
            DriveConstants.kRegularSpeed,
            true));
    // m_launcherWrist.setDefaultCommand(new ManualPositionSubsystem(m_launcherWrist));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // XBOX 0
    // m_driverController
    //     .leftBumper()
    //     .onTrue(
    //         new TeleopSwerve(
    //             m_drivebase,
    //             m_driverController,
    //             translationAxis,
    //             strafeAxis,
    //             rotationAxis,
    //             true,
    //             DriveConstants.kSlowSpeed,
    //             true));

    // m_driverController
    //     .leftBumper()
    //     .onFalse(
    //         new TeleopSwerve(
    //             m_drivebase,
    //             m_driverController,
    //             translationAxis,
    //             strafeAxis,
    //             rotationAxis,
    //             true,
    //             DriveConstants.kRegularSpeed,
    //             true));

    // m_driverController.back().onTrue(new InstantCommand(m_drivebase::zeroGyroscope));

    // // Example of an automatic path generated to score in the B2 zone
    // m_driverController
    //     .a()
    //     .onTrue(
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(1.8252, 2.779, Rotation2d.fromDegrees(180)),
    //             new PathConstraints(3, 4, Units.degreesToRadians(360),
    // Units.degreesToRadians(540)),
    //             0.0,
    //             0.0));

    // // Example of an automatic path generated to pick up from the human player
    // m_driverController
    //     .b()
    //     .onTrue(
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(16.06056, 6.270, Rotation2d.fromDegrees(0)),
    //             new PathConstraints(3, 4, Units.degreesToRadians(360),
    // Units.degreesToRadians(540)),
    //             0.0,
    //             0.0));

    // m_driverController.b().onTrue(new TurnToAngle(m_drivebase, 30));

    // m_driverController
    //     .a()
    //     .onTrue(new InstantCommand(() -> m_drivebase.setAngleToSnap(AngleToSnap.BACKWARD)));
    // m_driverController
    //     .y()
    //     .onTrue(new InstantCommand(() -> m_drivebase.setAngleToSnap(AngleToSnap.FORWARD)));
    // m_driverController.x().onTrue(new InstantCommand(m_drivebase::toggleXWheels));

    m_operatorController.a().onTrue(new TurnToAngle(m_drivebase));

    m_operatorController.a().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH));
    // m_operatorController.b().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.IDLE));
    // m_operatorController.x().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.OFF));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void tuningInit() {
    m_drivebase.tuningInit();
    tuningAlert.set(true);
  }

  public void tuningPeriodic() {
    m_drivebase.tuningPeriodic();
  }

  public void infoInit() {
    m_drivebase.infoInit();
    infoAlert.set(true);
  }

  public void infoPeriodic() {
    m_drivebase.infoPeriodic();
  }

  public void realPeriodic() {
    m_drivebase.realPeriodic();
    Limelight.getInstance().realPeriodic();
  }

  public void periodic() {}
}
