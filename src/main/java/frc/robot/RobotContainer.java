// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utilities.LoggedDashboardChooser;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.TeleopSwerve;
import frc.robot.commands.superstructure.ManualMultiMotorPositionSubsystem;
import frc.robot.commands.superstructure.ManualPositionSubsystem;
import frc.robot.commands.superstructure.SetSuperstructureState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.ElevatorLift;
import frc.robot.subsystems.intake.IntakeFlywheel;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;
import frc.robot.subsystems.launcher.LauncherFlywheel;
import frc.robot.subsystems.launcher.LauncherHold;
import frc.robot.subsystems.launcher.LauncherSuperstructure;
import frc.robot.subsystems.launcher.LauncherWrist;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;
import frc.robot.subsystems.leds.LED;
import frc.robot.subsystems.leds.LED.LEDState;
import frc.robot.subsystems.swerve.SwerveDrive;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // // SWERVE CONTROLS \\
  private final int translationAxis = PS5Controller.Axis.kLeftY.value;
  private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  private final int rotationAxis = PS5Controller.Axis.kRightX.value;

  // CONTROLLERS \\
  public static final CommandPS5Controller m_driverController =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  // public static final CommandXboxController m_driverController =
      // new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);


  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  private SwerveDrive m_drivebase = SwerveDrive.getInstance();
  private IntakeSuperstructure m_intakeSuperstructure = IntakeSuperstructure.getInstance();
  private LauncherSuperstructure m_launcherSuperstructure = LauncherSuperstructure.getInstance();
  private LED m_led = LED.getInstance();

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser;

  // ALERTS \\

  public RobotContainer() {
    


    // NAMED COMMANDS FOR AUTO \\
    NamedCommands.registerCommand("intake", IntakeSuperstructure.getInstance().setSuperstructureState(IntakeSuperstructureState.INTAKING));
    NamedCommands.registerCommand("eject", new SetVoltageSubsystemState(IntakeFlywheel.getInstance(), IntakeFlywheelState.EJECTING));

    // autoChooser =
    //     new LoggedDashboardChooser<>(
    //         "Auto Picker", AutoBuilder.buildAutoChooser(), mainTab, 0, 0, 2, 1);
    // autoChooser.addOption("Center Command", new SequentialCommandGroup(new InstantCommand(() -> m_drivebase.updateEstimatorWithPose(new Pose2d(2, 0.76, Rotation2d.fromDegrees(0)))), 
    // new CenterNoteAuto()));



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
    ElevatorLift.getInstance().setDefaultCommand(new ManualMultiMotorPositionSubsystem(ElevatorLift.getInstance())); // TRIGGERS
    LauncherWrist.getInstance().setDefaultCommand(new ManualPositionSubsystem(LauncherWrist.getInstance())); // LEFT X
    IntakeWrist.getInstance().setDefaultCommand(new ManualPositionSubsystem(IntakeWrist.getInstance())); // BUMPERS
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // DRIVER CONTROLS \\


    m_driverController.R1().onTrue(new TeleopSwerve(
      m_drivebase,
      m_driverController,
      translationAxis,
      strafeAxis,
      rotationAxis,
      true,
      DriveConstants.kSprintSpeed,
      true));

    m_driverController.R1().onFalse(new TeleopSwerve(
      m_drivebase,
      m_driverController,
      translationAxis,
      strafeAxis,
      rotationAxis,
      true,
      DriveConstants.kRegularSpeed,
      true));

    m_driverController.create().onTrue(new InstantCommand(m_drivebase::zeroGyroscope));

    m_driverController.options().onTrue(new InstantCommand(m_drivebase::toggleXWheels));

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

    /// LEDS /////
    m_driverController.pov(0).onTrue(new InstantCommand(() -> m_led.setState(LEDState.OFF)));
    m_driverController.pov(90).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE)));  //
    m_driverController.pov(180).onTrue(new InstantCommand(() -> m_led.setState(LEDState.ORANGE_WIPE))); // Can switch to other colors
    m_driverController.pov(270).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE_WIPE))); //
    
    ///// INTAKE /////
    m_operatorController.a().onTrue(new SetSuperstructureState(m_intakeSuperstructure, IntakeSuperstructureState.INTAKING)
    .alongWith(new SetSuperstructureState(m_launcherSuperstructure, LauncherSuperstructureState.INTAKING)));
    m_operatorController.b().onTrue(new SetSuperstructureState(m_intakeSuperstructure, IntakeSuperstructureState.STOWED)
    .alongWith(new SetSuperstructureState(m_launcherSuperstructure, LauncherSuperstructureState.OFF)));
    m_operatorController.x().onTrue(new SetSuperstructureState(m_intakeSuperstructure, IntakeSuperstructureState.AMP_PREPARE));
    // m_operatorController.x().onFalse(new SetVoltageSubsystemState(IntakeFlywheel.getInstance(), IntakeFlywheelState.EJECTING));   
    // .andThen(new WaitCommand(1)
    // .andThen(new SetSuperstructureState(m_intakeSuperstructure, IntakeSuperstructureState.STOWED))));
    m_operatorController.y().onTrue(new SetVoltageSubsystemState(IntakeFlywheel.getInstance(), IntakeFlywheelState.EJECTING));
    m_operatorController.y().onFalse(new SetVoltageSubsystemState(IntakeFlywheel.getInstance(), IntakeFlywheelState.OFF));

    m_operatorController.start().onTrue(new SetSuperstructureState(m_intakeSuperstructure, IntakeSuperstructureState.TRAVEL));
  
    m_operatorController.pov(0).onTrue(new SetSuperstructureState(m_launcherSuperstructure, IntakeSuperstructureState.LAUNCHING));
    m_operatorController.pov(180).onTrue(new SetSuperstructureState(m_launcherSuperstructure, IntakeSuperstructureState.DOWNOFF));
  
    ///// LAUNCH /////
    m_operatorController.pov(90).onTrue(new SetVelocitySubsystemState(LauncherFlywheel.getInstance(), LauncherFlywheelState.RUNNING));
    // .alongWith(new SetVoltageSubsystemState(LauncherHold.getInstance(), LauncherHoldState.LAUNCHING)));
    m_operatorController.pov(270).onTrue(new SetVelocitySubsystemState(LauncherFlywheel.getInstance(), LauncherFlywheelState.OFF));
    // .alongWith(new SetVoltageSubsystemState(LauncherHold.getInstance(), LauncherHoldState.OFF)));
    // m_operatorController.pov(90).onTrue(new SetSuperstructureState(m_launcherSuperstructure, LauncherSuperstructureState.RUNNING));
    // m_operatorController.pov(90).onFalse(new SetSuperstructureState(m_launcherSuperstructure, LauncherSuperstructureState.OFF));
    m_operatorController.pov(270).onTrue(new SetVoltageSubsystemState(LauncherHold.getInstance(), LauncherHoldState.OFF));
    m_operatorController.pov(90).onFalse(new SetVoltageSubsystemState(LauncherHold.getInstance(), LauncherHoldState.LAUNCHING));
}

  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return new InstantCommand();
  }
}
