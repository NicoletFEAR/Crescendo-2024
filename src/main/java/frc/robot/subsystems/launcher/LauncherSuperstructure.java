package frc.robot.subsystems.launcher;

import java.util.HashMap;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.utilities.PolarCoordinate;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetLedState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.waits.WaitForLaunchNote;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.leds.LED.LEDState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class LauncherSuperstructure extends SuperstructureSubsystem {

  private boolean m_noteInLauncher = false;

  private DigitalInput m_launcherBeamBreak;

  private static LauncherSuperstructure m_instance = null;

  public LauncherSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);

    m_launcherBeamBreak = new DigitalInput(LauncherConstants.kLaunchBeamBreakId);
  }

  public static LauncherSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new LauncherSuperstructure(LauncherSuperstructureState.STOW, "Launcher");
    }

    return m_instance;
  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    LauncherSuperstructureState launcherDesiredState = (LauncherSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();
    if (launcherDesiredState == LauncherSuperstructureState.STOW) {
      outputCommand.addCommands(
        new SetLedState(LEDState.GREEN_BLINKING),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState))
          .alongWith(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)),
        new SetLedState(LEDState.GREEN)
      );
    } else if (launcherDesiredState == LauncherSuperstructureState.RUNNING) {
      outputCommand.addCommands(
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.THRU_LAUNCHER_INTAKING));
        new WaitCommand(.02);
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.OFF);
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)
      );
    } else if (launcherDesiredState == LauncherSuperstructureState.THRU_INTAKE_INTAKING) {
      outputCommand.addCommands(
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState)),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)
      );
    }
     else {
      outputCommand.addCommands(
        new SetLedState(LEDState.REVVING),
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState)),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState),
        new SetLedState(LEDState.GREEN_FLASHING)
      );
    }
 
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = launcherDesiredState));

    if (launcherDesiredState == LauncherSuperstructureState.THRU_INTAKE_INTAKING) {
      outputCommand.addCommands(
        new SetLedState(LEDState.GREEN_BLINKING),
        new WaitForLaunchNote(),
        new WaitCommand(.01),
        setSuperstructureState(LauncherSuperstructureState.STOW)
        .alongWith(RobotContainer.m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.STOWED)),
        new SetLedState(LEDState.BLUE)
      );
    }

    return outputCommand;
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

    // if (m_desiredState == LauncherSuperstructureState.THRU_INTAKE_INTAKING && !m_launcherBeamBreak.get() && m_noteInLauncher == false) {
    //   new SequentialCommandGroup(
    //   new WaitCommand(.01),
    //   setSuperstructureState(LauncherSuperstructureState.STOW)
    //   .alongWith(IntakeSuperstructure.getInstance().setSuperstructureState(IntakeSuperstructureState.STOWED))).schedule();
    //   m_noteInLauncher = true;
    // }

    if (!m_launcherBeamBreak.get() && !m_noteInLauncher) {
      m_noteInLauncher = true;
    } if (m_launcherBeamBreak.get() && m_noteInLauncher) {
      m_noteInLauncher = false;
    }

    SmartDashboard.putBoolean("note in launcher", m_noteInLauncher);
    SmartDashboard.putBoolean("launcher beam break", m_launcherBeamBreak.get());
    
  }

  public enum LauncherSuperstructureState implements SuperstructureState {
    STOW(
        LauncherFlywheelState.OFF,
        LauncherWristState.DOWN,
        LauncherHoldState.OFF,
        "Off"),
    IDLE(
        LauncherFlywheelState.IDLE,
        LauncherWristState.DOWN,
        LauncherHoldState.OFF,
        "Idle"),
    RUNNING( // arbitrary testing speed
        LauncherFlywheelState.RUNNING,
        LauncherWristState.UP,
        LauncherHoldState.LAUNCHING,
        "Running"),
    THRU_INTAKE_INTAKING(
        LauncherFlywheelState.OFF,
        LauncherWristState.LAUNCH,
        LauncherHoldState.THRU_INTAKE_INTAKING,
        "Intaking"),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      LauncherHoldState.OFF,
      "Transition"),
    PODIUM( // use when against base of podium
      LauncherFlywheelState.PODIUM,
      LauncherWristState.PODIUM,
      LauncherHoldState.LAUNCHING,
      "Subwoofer"
    ),
    WING_NOTE_1( // use when against base of podium
      LauncherFlywheelState.WING_NOTE_1,
      LauncherWristState.WING_NOTE_1,
      LauncherHoldState.LAUNCHING,
      "Wing Note 1"
    ),
    WING_NOTE_2( // use when against base of podium
      LauncherFlywheelState.WING_NOTE_2,
      LauncherWristState.WING_NOTE_2,
      LauncherHoldState.LAUNCHING,
      "Wing Note 2"
    ),
    WING_NOTE_3( // use when against base of podium
      LauncherFlywheelState.WING_NOTE_3,
      LauncherWristState.WING_NOTE_3,
      LauncherHoldState.LAUNCHING,
      "Wing Note 3"
    ),
    FIELD_BASED_PREP(
      LauncherFlywheelState.FIELD_BASED_VELOCITY,
      LauncherWristState.FIELD_BASED_PITCH,
      LauncherHoldState.OFF,
      "Field Based Prep"
    ),
    FIELD_BASED_LAUNCH(
      LauncherFlywheelState.FIELD_BASED_VELOCITY,
      LauncherWristState.FIELD_BASED_PITCH,
      LauncherHoldState.LAUNCHING,
      "Field Based Launch"
    ),
    SUBWOOFER( // use when against base of speaker
      LauncherFlywheelState.SUBWOOFER,
      LauncherWristState.SUBWOOFER,
      LauncherHoldState.LAUNCHING,
      "Subwoofer"
    );


    public LauncherFlywheelState launcherFlywheelState;
    public LauncherWristState launcherWristState;
    public LauncherHoldState launcherHoldState;
    public String name;

    private LauncherSuperstructureState(
        LauncherFlywheelState launcherFlywheelState,
        LauncherWristState launcherWristState,
        LauncherHoldState launcherHoldState,
        String name) {
      this.launcherFlywheelState = launcherFlywheelState;
      this.launcherWristState = launcherWristState;
      this.launcherHoldState = launcherHoldState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }

  public static final class LauncherConstants {

    
    public static final int kWristCANCoderId = 4;
    public static final int kLaunchBeamBreakId = 0;


    // IN METERS
    public static final TreeMap<Double, Double> kDistanceRPMMap = new TreeMap<>();
    static {
      kDistanceRPMMap.put(0.0, 1000.0);
      kDistanceRPMMap.put(1.0, 2000.0);
      kDistanceRPMMap.put(1.5, 2500.0);
      kDistanceRPMMap.put(2.0, 3000.0);
      kDistanceRPMMap.put(2.5, 3500.0);
      kDistanceRPMMap.put(3.0, 4000.0);
      kDistanceRPMMap.put(3.5, 4500.0);
      kDistanceRPMMap.put(4.0, 5000.0);
      kDistanceRPMMap.put(4.5, 5500.0);
      kDistanceRPMMap.put(5.0, 6000.0);
      kDistanceRPMMap.put(5.5, 6500.0);
    }

    // input = theta and r
    // output = angle in degrees
    public static final HashMap<PolarCoordinate, Double> kDistancePitchMap = new HashMap<>();
    static {
      kDistancePitchMap.put(new PolarCoordinate(0.0, 1.25), 75.0);
      kDistancePitchMap.put(new PolarCoordinate(10.0, 1.25), 80.0);
      kDistancePitchMap.put(new PolarCoordinate(20.0, 1.25), 85.0);
      kDistancePitchMap.put(new PolarCoordinate(30.0, 1.25), 90.0);
      kDistancePitchMap.put(new PolarCoordinate(40.0, 1.25), 90.0);
      kDistancePitchMap.put(new PolarCoordinate(0.0, 2.0), 60.0);
      kDistancePitchMap.put(new PolarCoordinate(10.0, 2.0), 62.5);
      kDistancePitchMap.put(new PolarCoordinate(20.0, 2.0), 65.0);
      kDistancePitchMap.put(new PolarCoordinate(30.0, 2.0), 67.5);
      kDistancePitchMap.put(new PolarCoordinate(40.0, 2.0), 70.0);
      kDistancePitchMap.put(new PolarCoordinate(0.0, 3.0), 55.0);
      kDistancePitchMap.put(new PolarCoordinate(10.0, 3.0), 57.5);
      kDistancePitchMap.put(new PolarCoordinate(20.0, 3.0), 60.0);
      kDistancePitchMap.put(new PolarCoordinate(30.0, 3.0), 62.5);
      kDistancePitchMap.put(new PolarCoordinate(40.0, 3.0), 65.0);
    }
  }
}
