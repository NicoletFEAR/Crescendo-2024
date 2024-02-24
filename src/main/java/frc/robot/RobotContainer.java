// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.TeleopSwerve;
import frc.robot.commands.superstructure.ManualMultiMotorPositionSubsystem;
import frc.robot.commands.superstructure.ManualPositionSubsystem;
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
  private LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  private IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  private LauncherHold m_launcherHold = LauncherHold.getInstance();
  private LED m_led = LED.getInstance();

  // SENDABLE CHOOSER \\
  public static SendableChooser<Command> autoChooser;

  // ALERTS \\

  public RobotContainer() {
    


    // NAMED COMMANDS FOR AUTO \\
    // NamedCommands.registerCommand("intake", IntakeSuperstructure.getInstance().setSuperstructureState(IntakeSuperstructureState.INTAKING));
    // NamedCommands.registerCommand("eject", new SetVoltageSubsystemState(IntakeFlywheel.getInstance(), IntakeFlywheelState.EJECTING));

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

    /// LEDS /////
    m_driverController.pov(0).onTrue(new InstantCommand(() -> m_led.setState(LEDState.OFF)));
    m_driverController.pov(90).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE)));  //
    m_driverController.pov(180).onTrue(new InstantCommand(() -> m_led.setState(LEDState.ORANGE_WIPE))); // Can switch to other colors
    m_driverController.pov(270).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE_WIPE))); //
    
    ///// INTAKE /////
    m_operatorController.a().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.INTAKING)
    .alongWith(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.INTAKING)));
    m_operatorController.b().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.STOWED)
    .alongWith(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.OFF)));
    m_operatorController.x().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.AMP_PREPARE));
    // m_operatorController.x().onFalse(new SetVoltageSubsystemState(IntakeFlywheel.getInstance(), IntakeFlywheelState.EJECTING));   
    // .andThen(new WaitCommand(1)
    // .andThen(new SetSuperstructureState(m_intakeSuperstructure, IntakeSuperstructureState.STOWED))));
    m_operatorController.y().onTrue(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.EJECTING));
    m_operatorController.y().onFalse(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.OFF));

    m_operatorController.start().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL));
  
    ///// LAUNCH /////
    m_operatorController.pov(90).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING));
    // .alongWith(new SetVoltageSubsystemState(LauncherHold.getInstance(), LauncherHoldState.LAUNCHING)));
    m_operatorController.pov(270).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.OFF));
    // .alongWith(new SetVoltageSubsystemState(LauncherHold.getInstance(), LauncherHoldState.OFF)));
    // m_operatorController.pov(90).onTrue(new SetSuperstructureState(m_launcherSuperstructure, LauncherSuperstructureState.RUNNING));
    // m_operatorController.pov(90).onFalse(new SetSuperstructureState(m_launcherSuperstructure, LauncherSuperstructureState.OFF));
    m_operatorController.pov(270).onTrue(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF));
    m_operatorController.pov(90).onFalse(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.LAUNCHING));
}

  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return new InstantCommand();
  }
}
