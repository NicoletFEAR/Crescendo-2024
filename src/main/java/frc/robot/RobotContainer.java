// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utilities.Alert;
import frc.lib.utilities.LoggedDashboardChooser;
import frc.lib.utilities.ShuffleboardButton;
import frc.lib.utilities.Alert.AlertType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.superstructure.IntakeFlywheelBeamBreakStop;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

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
  public static ShuffleboardTab positionMechTuningTab =
      kTuningMode ? Shuffleboard.getTab("Position Mech Tuning") : null;
  public static ShuffleboardTab velocityMechTuningTab =
      kTuningMode ? Shuffleboard.getTab("Velocity Mech Tuning") : null;

  public static ShuffleboardButton m_applyPositionMechConfigs = kTuningMode ? new ShuffleboardButton("Apply Position Mech Configs", false, positionMechTuningTab, BuiltInWidgets.kToggleButton , null, 6, 0) : null;
  public static ShuffleboardButton m_applyVelocityMechConfigs = kTuningMode ? new ShuffleboardButton("Apply Velocity Mech Configs", false, velocityMechTuningTab, BuiltInWidgets.kToggleButton , null, 4, 0) : null;
  public static ShuffleboardButton m_goToPosition = kTuningMode ? new ShuffleboardButton("Go To Position", false, positionMechTuningTab, BuiltInWidgets.kToggleButton , null, 7, 0) : null;
  public static ShuffleboardButton m_goToVelocity = kTuningMode ? new ShuffleboardButton("Go To Velocity", false, velocityMechTuningTab, BuiltInWidgets.kToggleButton , null, 5, 0) : null;      

  // SUBSYSTEMS \\
  // private LauncherSuperstructure m_launcherSuperstructure = LauncherSuperstructure.getInstance();
  // private FalconTestingStateMachine m_falconTesting = FalconTestingStateMachine.getInstance();
  private IntakeSuperstructure m_intakeSuperstructure = IntakeSuperstructure.getInstance();

  // SENDABLE CHOOSER \\
  public static LoggedDashboardChooser<Command> autoChooser;

  // ALERTS \\
  private Alert tuningAlert =
      new Alert("Tuning Mode Activated, expect decreased network performance.", AlertType.INFO);

  public RobotContainer() {
    // m_launcherWrist.setDefaultCommand(new ManualPositionSubsystem(m_launcherWrist));
    // m_falconTesting.setDefaultCommand(new ManualTalonFXPositionSubsystem(m_falconTesting));
    m_intakeSuperstructure.setDefaultCommand(new IntakeFlywheelBeamBreakStop(m_intakeSuperstructure));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // m_operatorController.a().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH));
    // m_operatorController.b().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.IDLE));
    // m_operatorController.x().onTrue(m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.OFF));

    m_operatorController.a()
      .onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.IN))
      .onFalse(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.OFF));
    m_operatorController.b()
      .onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.OUT))
      .onFalse(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.OFF));
    m_operatorController.y()
      .onTrue(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.AMP))
      .onFalse(m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.OFF));

    // m_operatorController.a().onTrue(new InstantCommand(() -> LauncherFlywheel.getInstance().setDesiredState(LauncherFlywheelState.IDLE)));
    // m_operatorController.x().onTrue(new InstantCommand(() -> LauncherFlywheel.getInstance().setDesiredState(LauncherFlywheelState.OFF)));
    // m_operatorController.y().onTrue(new InstantCommand(() -> LauncherFlywheel.getInstance().setDesiredState(LauncherFlywheelState.RUNNING)));
    // m_operatorController.b().onTrue(new InstantCommand(() -> LauncherFlywheel.getInstance().setDesiredState(LauncherFlywheelState.FAST)));
    
    // m_operatorController.pov(0).onTrue(new SetTalonFXPositionSubsystemState(m_falconTesting, FalconTestingState.REALLY_REALLY_UP));
    // m_operatorController.pov(90).onTrue(new SetTalonFXPositionSubsystemState(m_falconTesting, FalconTestingState.REALLY_UP));
    // m_operatorController.pov(180).onTrue(new SetTalonFXPositionSubsystemState(m_falconTesting, FalconTestingState.UP));
    // m_operatorController.pov(270).onTrue(new SetTalonFXPositionSubsystemState(m_falconTesting, FalconTestingState.DOWN));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void tuningInit() {
    tuningAlert.set(true);
  }

  public void tuningPeriodic() {
  }

  public void infoInit() {
  }

  public void infoPeriodic() {
  }

  public void realPeriodic() {
  }

  public void periodic() {}
}
