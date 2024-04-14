package frc.robot.subsystems.launcher;

import java.util.HashMap;
import java.util.TreeMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.templates.SuperstructureSubsystem;
import frc.lib.utilities.PolarCoordinate;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetLEDState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.waits.WaitForNoLaunchNote;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;

public class LauncherSuperstructure extends SuperstructureSubsystem {

  private boolean m_noteInLauncher = false;

  private DigitalInput m_launcherBeamBreak;

  private static LauncherSuperstructure m_instance = null;

  private GenericEntry beamBreakEntry;

  public LauncherSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);

    beamBreakEntry = RobotContainer.mainTab.add("Note In Launcher", m_noteInLauncher).withPosition(1, 1).withSize(1, 4).getEntry();

    // SmartDashboard.putBoolean("note in launcher", m_noteInLauncher);

    m_launcherBeamBreak = new DigitalInput(LauncherConstants.kLaunchBeamBreakId);
  }

  public static LauncherSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new LauncherSuperstructure(LauncherSuperstructureState.STOWED, "Launcher");
    }

    return m_instance;
  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    LauncherSuperstructureState launcherDesiredState = (LauncherSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(
      new InstantCommand(() -> {m_currentState = LauncherSuperstructureState.TRANSITION; m_desiredState = launcherDesiredState;}));

    switch (launcherDesiredState) {
      case STOWED:
        handleStowCommand(launcherDesiredState, outputCommand);
        break;
      case IDLE:
        handleIdleCommand(launcherDesiredState, outputCommand);
        break;
      case INTAKE_TO_LAUNCH:
        handleThruIntakeCommand(launcherDesiredState, outputCommand);
        break;
      case FIELD_BASED_PREP:
        handleFieldBasedPrep(launcherDesiredState, outputCommand);
        break;
      default:
        handleDefaultCommand(launcherDesiredState, outputCommand);
        break;
    }

    outputCommand.addCommands(new InstantCommand(() -> m_currentState = launcherDesiredState));

    return outputCommand;
  }

  private void handleDefaultCommand(LauncherSuperstructureState launcherDesiredState, SequentialCommandGroup outputCommand) {
      outputCommand.addCommands(
        new SetLEDState(LEDState.GREEN_LAUNCHER_LEDS),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState)),
        new WaitCommand(.02),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)
          .alongWith(
            new InstantCommand(() -> RobotContainer.m_intakeHold.setState(IntakeHoldState.INTAKING))
          ),
        new WaitForNoLaunchNote(),
        new SetLEDState(LEDState.GREEN_FLASHING, 1.0, LEDState.TEAL_STOW),
        new InstantCommand(() -> RobotContainer.m_intakeHold.setState(IntakeHoldState.OFF)),
        new WaitCommand(.04)
      );
      
      
  }

  private void handleStowCommand(LauncherSuperstructureState launcherDesiredState, SequentialCommandGroup outputCommand) {
    outputCommand.addCommands(
      new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState).unless(this::getNoteInLauncher)
        .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState))
        .alongWith(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState))
        .andThen(new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, LauncherFlywheelState.IDLE).onlyIf(() -> RobotContainer.m_launcherFlywheel.getVelocity()[0] > LauncherFlywheelState.IDLE.getVelocity()[0] && getNoteInLauncher()))
    );
  }

  private void handleFieldBasedPrep(LauncherSuperstructureState launcherDesiredState, SequentialCommandGroup outputCommand) {
    outputCommand.addCommands(
      new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
        .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState))
        .alongWith(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState))
    );
  }

  private void handleIdleCommand(LauncherSuperstructureState launcherDesiredState, SequentialCommandGroup outputCommand) {
    outputCommand.addCommands(
      new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
        .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState))
        .alongWith(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState))
    );
  }

  // private void handlePrepareCommand(LauncherSuperstructureState launcherDesiredState, SequentialCommandGroup outputCommand) {
  //   outputCommand.addCommands(
  //     new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
  //       .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState))
  //   );
  // }

  private void handleThruIntakeCommand(LauncherSuperstructureState launcherDesiredState, SequentialCommandGroup outputCommand) {
    outputCommand.addCommands(
      new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
        .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState)),
      new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)
    );
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return LauncherSuperstructureState.TRANSITION;
  }

  public boolean getNoteInLauncher() {
    return m_noteInLauncher;
  }

  @Override
  public void superstructurePeriodic() {
    if (!m_launcherBeamBreak.get() && !m_noteInLauncher) {
      m_noteInLauncher = true;
    } if (m_launcherBeamBreak.get() && m_noteInLauncher) {
      m_noteInLauncher = false;
    }

    // m_noteInLauncher = SmartDashboard.getBoolean("note in launcher", m_noteInLauncher);

    beamBreakEntry.setBoolean(m_noteInLauncher);
    
    if (Constants.kInfoMode) {
      SmartDashboard.putBoolean(m_name + "/" + "launcher beam break", m_launcherBeamBreak.get());
    }

    
  }

  public enum LauncherSuperstructureState implements SuperstructureState {
    STOWED(
      LauncherFlywheelState.OFF,
      LauncherWristState.DOWN,
      LauncherHoldState.OFF),
    IDLE(
      LauncherFlywheelState.IDLE,
      LauncherWristState.DOWN,
      LauncherHoldState.OFF),
    RUNNING( // arbitrary testing speed
      LauncherFlywheelState.RUNNING,
      LauncherWristState.DOWN,
      LauncherHoldState.LAUNCHING),
    THRU_INTAKE_INTAKING(
      LauncherFlywheelState.OFF,
      LauncherWristState.DOWN,
      LauncherHoldState.THRU_INTAKE_INTAKING),
    INTAKE_TO_LAUNCH(
      LauncherFlywheelState.OFF,
      LauncherWristState.DOWN,
      LauncherHoldState.THRU_INTAKE_INTAKING),
    LAUNCH_TO_INTAKE(
      LauncherFlywheelState.INTAKING,
      LauncherWristState.DOWN,
      LauncherHoldState.THRU_LAUNCHER_INTAKING),
    PODIUM( // use when against base of podium
      LauncherFlywheelState.PODIUM,
      LauncherWristState.PODIUM,
      LauncherHoldState.LAUNCHING),
    WING_NOTE_1( 
      LauncherFlywheelState.WING_NOTE_1,
      LauncherWristState.WING_NOTE_1,
      LauncherHoldState.LAUNCHING),
    WING_NOTE_2( 
      LauncherFlywheelState.WING_NOTE_2,
      LauncherWristState.WING_NOTE_2,
      LauncherHoldState.LAUNCHING),
    WING_NOTE_3( 
      LauncherFlywheelState.WING_NOTE_3,
      LauncherWristState.WING_NOTE_3,
      LauncherHoldState.LAUNCHING),
    LAUNCH_POS_1(
      LauncherFlywheelState.LAUNCH_POS_1,
      LauncherWristState.LAUNCH_POS_1,
      LauncherHoldState.LAUNCHING),
    LAUNCH_POS_2(
      LauncherFlywheelState.LAUNCH_POS_2,
      LauncherWristState.LAUNCH_POS_2,
      LauncherHoldState.LAUNCHING),
    LAUNCH_POS_3(
      LauncherFlywheelState.LAUNCH_POS_3,
      LauncherWristState.LAUNCH_POS_3,
      LauncherHoldState.LAUNCHING),
    POOP_POS_1( 
      LauncherFlywheelState.POOP_POS_1,
      LauncherWristState.POOP_POS_1,
      LauncherHoldState.LAUNCHING),
    POOP_POS_2( 
      LauncherFlywheelState.POOP_POS_2,
      LauncherWristState.POOP_POS_2,
      LauncherHoldState.LAUNCHING),
    POOP_POS_3(
      LauncherFlywheelState.POOP_POS_3,
      LauncherWristState.POOP_POS_3,
      LauncherHoldState.LAUNCHING),
    FIELD_BASED_PREP(
      LauncherFlywheelState.FIELD_BASED_VELOCITY,
      LauncherWristState.FIELD_BASED_PITCH,
      LauncherHoldState.OFF),
    FIELD_BASED_LAUNCH(
      LauncherFlywheelState.FIELD_BASED_VELOCITY,
      LauncherWristState.FIELD_BASED_PITCH,
      LauncherHoldState.LAUNCHING),
    PASS(
      LauncherFlywheelState.PASS,
      LauncherWristState.PASS,
      LauncherHoldState.PASS),
    SUBWOOFER(
      LauncherFlywheelState.SUBWOOFER,
      LauncherWristState.SUBWOOFER,
      LauncherHoldState.LAUNCHING),
    SUBWOOFER_PREPARE(
      LauncherFlywheelState.SUBWOOFER,
      LauncherWristState.SUBWOOFER,
      LauncherHoldState.OFF),
    TESTING(
      LauncherFlywheelState.FIELD_BASED_VELOCITY,
      LauncherWristState.TESTING,
      LauncherHoldState.LAUNCHING),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      LauncherHoldState.OFF);


    public LauncherFlywheelState launcherFlywheelState;
    public LauncherWristState launcherWristState;
    public LauncherHoldState launcherHoldState;

    private LauncherSuperstructureState(
        LauncherFlywheelState launcherFlywheelState,
        LauncherWristState launcherWristState,
        LauncherHoldState launcherHoldState) {
      this.launcherFlywheelState = launcherFlywheelState;
      this.launcherWristState = launcherWristState;
      this.launcherHoldState = launcherHoldState;
    }

    @Override
    public String getName() {
      return name();
    }
  }

  public static final class LauncherConstants {

    
    public static final int kWristCANCoderId = 4;
    public static final int kLaunchBeamBreakId = 0;


    // IN METERS
    public static final TreeMap<Double, Double> kAmpDistanceRPMMap = new TreeMap<>();
    static {
      kAmpDistanceRPMMap.put(0.0, 3000.0);
      kAmpDistanceRPMMap.put(1.0, 3000.0);
      kAmpDistanceRPMMap.put(2.0, 3000.0);
      kAmpDistanceRPMMap.put(3.0, 3500.0);
      kAmpDistanceRPMMap.put(4.0, 4000.0);
      kAmpDistanceRPMMap.put(5.0, 4500.0);
      kAmpDistanceRPMMap.put(6.0, 5000.0);
      kAmpDistanceRPMMap.put(7.0, 5500.0);
      kAmpDistanceRPMMap.put(8.0, 6000.0);
      kAmpDistanceRPMMap.put(9.0, 6000.0);
      kAmpDistanceRPMMap.put(10.0, 6000.0);
      kAmpDistanceRPMMap.put(11.0, 6000.0);
      kAmpDistanceRPMMap.put(12.0, 6000.0);
      kAmpDistanceRPMMap.put(40.0, 6000.0);
    }

    public static final TreeMap<Double, Double> kAmpDistancePitchMap = new TreeMap<>();
    static {
      kAmpDistancePitchMap.put(0.0, 0.0);
      kAmpDistancePitchMap.put(1.0, 10.0);
      kAmpDistancePitchMap.put(2.0, 40.0);
      kAmpDistancePitchMap.put(3.0, 40.0);
      kAmpDistancePitchMap.put(4.0, 40.0);
      kAmpDistancePitchMap.put(5.0, 40.0);
      kAmpDistancePitchMap.put(6.0, 40.0);
      kAmpDistancePitchMap.put(7.0, 50.0);
      kAmpDistancePitchMap.put(8.0, 60.0);
      kAmpDistancePitchMap.put(9.0, 70.0);
      kAmpDistancePitchMap.put(10.0, 70.0);
      kAmpDistancePitchMap.put(11.0, 70.0);
      kAmpDistancePitchMap.put(12.0, 70.0);
      kAmpDistancePitchMap.put(40.0, 70.0);
    }

    // input = theta and r
    // output = angle in degrees

    // HAVE TO PUT IT IN ORDER
    // GROUPS OF R 
    // WITH THETA ORDERED LOWEST TO HIGHEST
    public static final HashMap<PolarCoordinate, Double> kDistancePitchMap = new HashMap<>();
    static {
      kDistancePitchMap.put(new PolarCoordinate(0.0, 0.1), 110.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 0.1), 110.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 1.5), 110.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 1.5), 110.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 1.75), 96.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 1.75), 96.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 2.0), 86.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 2.0), 86.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 2.25), 76.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 2.25), 76.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 2.5), 73.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 2.5), 73.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 2.75), 68.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 2.75), 68.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 3.0), 65.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 3.0), 65.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 3.25), 63.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 3.25), 63.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 3.5), 62.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 3.5), 62.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 3.75), 52.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 3.75), 52.0);

      kDistancePitchMap.put(new PolarCoordinate(0.0, 15.0), 52.0);
      kDistancePitchMap.put(new PolarCoordinate(90.0, 15.0), 52.0);
    }
  }
}
