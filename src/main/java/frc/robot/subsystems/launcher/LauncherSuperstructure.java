package frc.robot.subsystems.launcher;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class LauncherSuperstructure extends SuperstructureSubsystem {

  private LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  private LauncherWrist m_launcherWrist = LauncherWrist.getInstance();
  private LauncherHold m_launcherHold = LauncherHold.getInstance();

  private DigitalInput m_launcherBeamBreak;

  private static LauncherSuperstructure m_instance = null;

  public LauncherSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);

    m_launcherBeamBreak = new DigitalInput(LauncherConstants.kLaunchBeamBreakId);
  }

  public static LauncherSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new LauncherSuperstructure(LauncherSuperstructureState.OFF, "Launcher");
    }

    return m_instance;
  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    LauncherSuperstructureState launcherDesiredState = (LauncherSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(
      new SetVelocitySubsystemState(m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState)
      .alongWith(new SetPositionSubsystemState(m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState))
      .alongWith(new SetVoltageSubsystemState(m_launcherHold, launcherDesiredState.launcherHoldState))
    );
    // outputCommand.addCommands(new SetVoltageSubsystemState(m_launcherHold, launcherDesiredState.launcherHoldState));
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = launcherDesiredState));

    return outputCommand;
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return LauncherSuperstructureState.TRANSITION;
  }

  @Override
  public void superstructurePeriodic() {

    // if (m_currentState == LauncherSuperstructureState.INTAKE && m_launcherBeamBreak.get()) {
    //   setSuperstructureState(LauncherSuperstructureState.NOTE_IN_HOLD).schedule();
    // }


    // Logger.recordOutput(m_name + "/Superstructure/BeamBreak", m_launcherBeamBreak.get());
  }

  public enum LauncherSuperstructureState implements SuperstructureState {
    OFF( // should clean up
        LauncherFlywheelState.OFF,
        LauncherWristState.DOWN,
        LauncherHoldState.OFF,
        "Off"),
    IDLE(
        LauncherFlywheelState.IDLE,
        LauncherWristState.DOWN,
        LauncherHoldState.OFF,
        "Idle"),
    RUNNING(
        LauncherFlywheelState.RUNNING,
        LauncherWristState.UP,
        LauncherHoldState.LAUNCHING,
        "Running"),
    FAST(
        LauncherFlywheelState.FAST,
        LauncherWristState.DOWN,
        LauncherHoldState.LAUNCHING,
        "Fast"),
    INTAKING(
        LauncherFlywheelState.INTAKING,
        LauncherWristState.LAUNCH,
        LauncherHoldState.INTAKING,
        "Intaking"),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      LauncherHoldState.OFF,
      "Transition"),
    SUBWOOFER(
      LauncherFlywheelState.FAST,
      LauncherWristState.LAUNCH,
      LauncherHoldState.OFF,
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
    public static final int kLaunchBeamBreakId = 1;


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

    // IN DEGREES
    public static final TreeMap<Double, Double> kDistancePitchMap = new TreeMap<>();
    static {
      kDistancePitchMap.put(0.0, 90.0);
      kDistancePitchMap.put(1.0, 60.0);
      kDistancePitchMap.put(1.5, 55.0);
      kDistancePitchMap.put(2.0, 50.0);
      kDistancePitchMap.put(2.5, 45.0);
      kDistancePitchMap.put(3.0, 40.0);
      kDistancePitchMap.put(3.5, 35.0);
      kDistancePitchMap.put(4.0, 30.0);
      kDistancePitchMap.put(4.5, 25.0);
      kDistancePitchMap.put(5.0, 20.0);
      kDistancePitchMap.put(5.5, 15.0);
      kDistancePitchMap.put(6.0, 10.0);
    }
  }
}
