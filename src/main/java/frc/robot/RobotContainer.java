// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.TeleopSwerve;
import frc.robot.commands.drivebase.TurnToAngle;
import frc.robot.commands.sequential.FieldRelativeLaunch;
import frc.robot.commands.superstructure.ManualMultiMotorPositionSubsystem;
import frc.robot.commands.superstructure.ManualPositionSubsystem;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
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
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.leds.LED;
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

  public static IntakeSuperstructure m_intakeSuperstructure = IntakeSuperstructure.getInstance();
  public static IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  public static IntakeHold m_intakeHold = IntakeHold.getInstance();
  public static IntakeWrist m_intakeWrist = IntakeWrist.getInstance();
  public static ElevatorLift m_elevatorLift = ElevatorLift.getInstance();

  public static LauncherSuperstructure m_launcherSuperstructure = LauncherSuperstructure.getInstance();
  public static LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  public static LauncherHold m_launcherHold = LauncherHold.getInstance();
  public static LauncherWrist m_launcherWrist = LauncherWrist.getInstance();

    public static SwerveDrive m_drivebase = SwerveDrive.getInstance();

  
  public static LED m_led = LED.getInstance();
  
  // SENDABLE CHOOSER \\
  public static SendableChooser<Command> autoChooser;

  public RobotContainer() {

    // NAMED COMMANDS FOR AUTO \\

    // Use for auto
    NamedCommands.registerCommand("fastIntake", m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.FAST_BEAM_BREAK_INTAKING));
    NamedCommands.registerCommand("intakeDown", m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.DOWNOFF));
    NamedCommands.registerCommand("runLaunch", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING));

    NamedCommands.registerCommand("subwooferLaunch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.SUBWOOFER));

    NamedCommands.registerCommand("wingNote1LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.WING_NOTE_1)));
    NamedCommands.registerCommand("wingNote1Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.WING_NOTE_1));

    NamedCommands.registerCommand("wingNote2LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.WING_NOTE_2)));
    NamedCommands.registerCommand("wingNote2Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.WING_NOTE_2));

    NamedCommands.registerCommand("wingNote3LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.WING_NOTE_3)));
    NamedCommands.registerCommand("wingNote3Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.WING_NOTE_3));

    NamedCommands.registerCommand("launchPos1LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.LAUNCH_POS_1)));
    NamedCommands.registerCommand("launchPos1Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_POS_1));
  
    NamedCommands.registerCommand("launchPos2LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.LAUNCH_POS_2)));
    NamedCommands.registerCommand("launchPos2Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_POS_2));

    NamedCommands.registerCommand("launchPos3LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.LAUNCH_POS_3)));
    NamedCommands.registerCommand("launchPos3Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_POS_3));

    NamedCommands.registerCommand("poopPos1LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.POOP_POS_1)));
    NamedCommands.registerCommand("poopPos1Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.POOP_POS_1));
  
    NamedCommands.registerCommand("poopPos2LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.POOP_POS_2)));
    NamedCommands.registerCommand("poopPos2Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.POOP_POS_2));

    NamedCommands.registerCommand("poopPos3LaunchPrepare", new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.POOP_POS_3)));
    NamedCommands.registerCommand("poopPos3Launch", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.POOP_POS_3));

    NamedCommands.registerCommand("addGyroAmpSide", new InstantCommand(() -> m_drivebase.setGyroRequest(true, true)));
    NamedCommands.registerCommand("addGyroSourceSide", new InstantCommand(() -> m_drivebase.setGyroRequest(true, false)));

    NamedCommands.registerCommand("stow", m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)
    .andThen(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL)));


    
    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("None", new InstantCommand());

    autoChooser.addOption("Amp Side 4 Piece Wing", AutoBuilder.buildAuto("Amp Side 4 Piece Wing"));
    autoChooser.addOption("Amp Side 3 Piece Center", AutoBuilder.buildAuto("Amp Side 3 Piece Center"));
    // autoChooser.addOption("Amp Side 4 Piece Center", AutoBuilder.buildAuto("Amp Side 4 Piece Center"));
    // autoChooser.addOption("Amp Side 5 Piece Center", AutoBuilder.buildAuto("Amp Side 5 Piece Center"));
    autoChooser.addOption("Amp Side 2 Piece", AutoBuilder.buildAuto("Amp Side 2 Piece"));

    autoChooser.addOption("Front Side 2 Piece", AutoBuilder.buildAuto("Front Side 2 Piece"));
    // autoChooser.addOption("Front Side 3 Piece", AutoBuilder.buildAuto("Front Side 3 Piece"));
    // autoChooser.addOption("Front Side 4 Piece", AutoBuilder.buildAuto("Front Side 4 Piece"));
    // autoChooser.addOption("Front Side 5 Piece", AutoBuilder.buildAuto("Front Side 5 Piece"));

    // autoChooser.addOption("Source Side 4 Piece Wing", AutoBuilder.buildAuto("Source Side 4 Piece Wing"));
    // autoChooser.addOption("Source Side 3 Piece Center", AutoBuilder.buildAuto("Source Side 3 Piece Center"));
    // autoChooser.addOption("Source Side 4 Piece Center", AutoBuilder.buildAuto("Source Side 4 Piece Center"));
    // autoChooser.addOption("Source Side 5 Piece Center", AutoBuilder.buildAuto("Source Side 5 Piece Center"));
    autoChooser.addOption("Source Side 2 Piece", AutoBuilder.buildAuto("Source Side 2 Piece"));

    autoChooser.addOption("Center 5 Piece", AutoBuilder.buildAuto("Center 5 Piece"));

    mainTab.add(autoChooser).withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);

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
      DriveConstants.kSlowSpeed,
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

    m_driverController.options().onTrue(new InstantCommand(m_drivebase::resetAngleToAbsolute));

    m_driverController.cross().onTrue(new InstantCommand(m_drivebase::toggleXWheels));
    m_driverController.circle().whileTrue(new TurnToAngle(m_drivebase));

    
    ///// INTAKE /////
    m_operatorController.a().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.BEAM_BREAK_INTAKING));
    // m_operatorController.pov(0).onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL));

    m_operatorController.rightStick().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TOF_INTAKING));

    m_operatorController.b().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL));
    
    m_operatorController.x().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.AMP_PREPARE));

    m_operatorController.y().onTrue(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.EJECTING)
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.EJECTING)));
      // .andThen(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.THRU_INTAKE_EJECTING)));
      // .alongWith(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)));
      
    m_operatorController.y().onFalse(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.OFF)
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.OFF)));
      // .andThen(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF)));

    m_operatorController.start().onTrue(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.INTAKING)
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.INTAKING))
      .alongWith(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.THRU_INTAKE_INTAKING)));
      
    m_operatorController.start().onFalse(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.OFF)
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.OFF))
      .alongWith(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF)));


    m_operatorController.back().onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.CLIMB_PREPARE));

    m_operatorController.leftStick().onTrue(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.INTAKE_TO_LAUNCH));
    m_operatorController.leftStick().onFalse(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.OFF));
  
    ///// LAUNCH /////
    m_operatorController.pov(180).onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.SUBWOOFER));
    m_operatorController.pov(180).onFalse(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED));

    m_operatorController.pov(0).onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.PASS));
    m_operatorController.pov(0).onFalse(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED));

    m_operatorController.pov(270).onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.STOWED));
    m_operatorController.pov(270).onFalse(new FieldRelativeLaunch(LauncherSuperstructureState.PODIUM));

    m_operatorController.pov(90).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.OFF)
    .alongWith(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF)));
      
    // m_operatorController.pov(0).onTrue(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.EJECTING)
    //   .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.INTAKING)).3
    
    //   .alongWith(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.THRU_INTAKE_INTAKING)));
      
    // m_operatorController.pov(0).onFalse(new SetVoltageSubsystemState(m_intakeFlywheel, IntakeFlywheelState.OFF)
    //   .alongWith(new SetVoltageSubsystemState(m_intakeHold, IntakeHoldState.OFF))
    //   .alongWith(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF)));
    // m_operatorController.a().onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.SUBWOOFER));
    // m_operatorController.a().onFalse(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.OFF));
    
    // m_operatorController.b().onTrue(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.LAUNCHING));
    // m_operatorController.b().onFalse(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF));

    // m_operatorController.x().onTrue(new SetPositionSubsystemState(m_launcherWrist, LauncherWristState.SUBWOOFER));

    // m_operatorController.pov(0).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.PODIUM));
    // m_operatorController.pov(0).onFalse((new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.LAUNCHING)));

    ///// SIGNALS /////
    // m_operatorController.pov(270).onTrue(new SetLEDState(LEDState.BLUE_WIPE, 3));
    // m_operatorController.pov(90).onTrue(new SetLEDState(LEDState.ORANGE_WIPE, 100));
    // m_operatorController.pov(90).onFalse(new SetLEDState(LEDState.ORANGE_WIPE, -1));

    // m_operatorController.pov(90).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.RUNNING));
    // m_operatorController.pov(270).onTrue(new SetVelocitySubsystemState(m_launcherFlywheel, LauncherFlywheelState.OFF));
    // m_operatorController.pov(270).onTrue(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.OFF));
    // m_operatorController.pov(90).onFalse(new SetVoltageSubsystemState(m_launcherHold, LauncherHoldState.LAUNCHING));
}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}