// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.pathplanner.lib.auto.AutoBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utilities.LoggedDashboardChooser;
// import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.auto.CenterNoteAuto;
// import frc.robot.commands.drivebase.TeleopSwerve;
import frc.robot.commands.superstructure.ManualPositionSubsystem;
// import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.subsystems.intake.ElevatorLift;
// import frc.robot.subsystems.launcher.LauncherFlywheel;
// import frc.robot.subsystems.launcher.LauncherSuperstructure;
// import frc.robot.subsystems.launcher.LauncherWrist;
// import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
// import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;
// import frc.robot.subsystems.swerve.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // // SWERVE CONTROLS \\
  // private final int translationAxis = PS5Controller.Axis.kLeftY.value;
  // private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  // private final int rotationAxis = PS5Controller.Axis.kRightX.value;

  // CONTROLLERS \\
  public static final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  
  // private SwerveDrive m_drivebase = SwerveDrive.getInstance();
  // private IntakeSuperstructure m_IntakeSuperstructure = IntakeSuperstructure.getInstance();
  // private LauncherSuperstructure m_launcherSuperstructure = LauncherSuperstructure.getInstance();

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser;

  // ALERTS \\

  public RobotContainer() {
    ElevatorLift.getInstance().setDefaultCommand(new ManualPositionSubsystem(ElevatorLift.getInstance()));
    // LauncherWrist.getInstance().setDefaultCommand(new ManualPositionSubsystem(LauncherWrist.getInstance()));

    // NAMED COMMANDS FOR AUTO \\
    // you would never do this while following a path, its just to show how to implement

    // autoChooser =
    //     new LoggedDashboardChooser<>(
    //         "Auto Picker", AutoBuilder.buildAutoChooser(), mainTab, 0, 0, 2, 1);
    // autoChooser.addOption("Center Command", new SequentialCommandGroup(new InstantCommand(() -> m_drivebase.updateEstimatorWithPose(new Pose2d(2, 0.76, Rotation2d.fromDegrees(0)))), 
    // new CenterNoteAuto()));

    // CONFIGURE DEFAULT COMMANDS \\
    // m_drivebase.setDefaultCommand(
    //     new TeleopSwerve(
    //         m_drivebase,
    //         m_driverController,
    //         translationAxis,
    //         strafeAxis,
    //         rotationAxis,
    //         true,
    //         DriveConstants.kRegularSpeed,
    //         true));
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // DRIVER CONTROLS \\


    // m_driverController.R1().onTrue(new TeleopSwerve(
    //   m_drivebase,
    //   m_driverController,
    //   translationAxis,
    //   strafeAxis,
    //   rotationAxis,
    //   true,
    //   DriveConstants.kSprintSpeed,
    //   true));

    // m_driverController.R1().onFalse(new TeleopSwerve(
    //   m_drivebase,
    //   m_driverController,
    //   translationAxis,
    //   strafeAxis,
    //   rotationAxis,
    //   true,
    //   DriveConstants.kRegularSpeed,
    //   true));

    // m_driverController.create().onTrue(new InstantCommand(m_drivebase::zeroGyroscope));

    // m_operatorController.a().onTrue(new SetPositionSubsystemState(IntakeWrist.getInstance(), IntakeWristState.DOWN));
    // m_operatorController.b().onTrue(new SetPositionSubsystemState(IntakeWrist.getInstance(), IntakeWristState.AMP));

    // m_operatorController.x().onTrue(new SetPositionSubsystemState(ElevatorLift.getInstance(), ElevatorLiftState.DOWN));
    // m_operatorController.y().onTrue(new SetPositionSubsystemState(ElevatorLift.getInstance(), ElevatorLiftState.AMP));

    // m_operatorController.a().onTrue(new SetSuperstructureState(m_IntakeSuperstructure, IntakeSuperstructureState.IN));
    // m_operatorController.b().onTrue(new SetSuperstructureState(m_IntakeSuperstructure, IntakeSuperstructureState.AMP));
    // m_operatorController.x().onTrue(new SetSuperstructureState(m_IntakeSuperstructure, IntakeSuperstructureState.LAUNCHING));
    // m_operatorController.y().onTrue(new SetSuperstructureState(m_IntakeSuperstructure, IntakeSuperstructureState.OFF));


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

    // m_operatorController.a().onTrue(new TurnToAngle(m_drivebase));
    // m_operatorController.a().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.IDLE));
    // m_operatorController.b().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.FAST));
    // m_operatorController.x().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.RUNNING));
    // m_operatorController.y().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.OFF));

    // m_operatorController.a().onTrue(new SetVelocitySubsystemState(LauncherFlywheel.getInstance(), LauncherFlywheelState.FAST));
    // m_operatorController.b().onTrue(new SetVelocitySubsystemState(LauncherFlywheel.getInstance(), LauncherFlywheelState.RUNNING));
    // m_operatorController.x().onTrue(new SetVelocitySubsystemState(LauncherFlywheel.getInstance(), LauncherFlywheelState.IDLE));
    // m_operatorController.y().onTrue(new SetVelocitySubsystemState(LauncherFlywheel.getInstance(), LauncherFlywheelState.OFF));
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
  }
}
