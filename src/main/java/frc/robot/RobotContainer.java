// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.subsystems.intake.IntakeHold;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeWrist;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
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
  public static final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);


  // SHUFFLEBOARD TABS \\
  public static ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  public static SwerveDrive m_drivebase = SwerveDrive.getInstance();

  public static IntakeSuperstructure m_intakeSuperstructure = IntakeSuperstructure.getInstance();
  public static IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  public static IntakeHold m_intakeHold = IntakeHold.getInstance();
  public static IntakeWrist m_intakeWrist = IntakeWrist.getInstance();
  public static ElevatorLift m_elevatorLift = ElevatorLift.getInstance();

  public static LauncherSuperstructure m_launcherSuperstructure = LauncherSuperstructure.getInstance();
  public static LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  public static LauncherHold m_launcherHold = LauncherHold.getInstance();
  public static LauncherWrist m_launcherWrist = LauncherWrist.getInstance();
  
  public static LED m_led = LED.getInstance();
  
  // SENDABLE CHOOSER \\
  public static SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\
    NamedCommands.registerCommand("intake", m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.AUTO_INTAKING));
    NamedCommands.registerCommand("subwooferLaunch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.SUBWOOFER));
    NamedCommands.registerCommand("wn1Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.WING_NOTE_1));
    NamedCommands.registerCommand("wn2Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.WING_NOTE_2));
    NamedCommands.registerCommand("wn3Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.WING_NOTE_3));

    autoChooser = AutoBuilder.buildAutoChooser();

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
    
    m_elevatorLift.setDefaultCommand(new ManualMultiMotorPositionSubsystem(m_elevatorLift)); // TRIGGERS
    m_launcherWrist.setDefaultCommand(new ManualPositionSubsystem(m_launcherWrist)); // LEFT X
    m_intakeWrist.setDefaultCommand(new ManualPositionSubsystem(m_intakeWrist)); // BUMPERS
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

    m_driverController.L1().onTrue(new TeleopSwerve(
      m_drivebase,
      m_driverController,
      translationAxis,
      strafeAxis,
      rotationAxis,
      true,
      DriveConstants.kRegularSpeed,
      false));

    m_driverController.L1().onFalse(new TeleopSwerve(
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
    m_driverController.pov(90).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE)));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> m_led.setState(LEDState.ORANGE_WIPE)));
    m_driverController.pov(270).onTrue(new InstantCommand(() -> m_led.setState(LEDState.BLUE_WIPE)));
    
    ///// INTAKE /////
    m_operatorController.a().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TELE_INTAKING));

    m_operatorController.b().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.STOWED));
    
    m_operatorController.x().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.AMP_PREPARE));

    m_operatorController.y().onTrue(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.EJECTING)
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.EJECTING)));
    m_operatorController.y().onFalse(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.OFF)
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.OFF)));

    m_operatorController.start().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL));
  
    ///// LAUNCH /////
    m_operatorController.pov(180).onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.SUBWOOFER));
    m_operatorController.pov(180).onFalse(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOW));

    m_operatorController.pov(0).onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.PODIUM));
    m_operatorController.pov(0).onFalse(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOW));

    m_operatorController.pov(90).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING));
    m_operatorController.pov(270).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.OFF));
    m_operatorController.pov(270).onTrue(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF));
    m_operatorController.pov(90).onFalse(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.LAUNCHING));
}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}