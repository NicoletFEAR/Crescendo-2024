package frc.robot.subsystems.launcher;

import java.util.HashMap;
import java.util.TreeMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.utilities.PolarCoordinate;
import frc.robot.Constants;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class LauncherSuperstructure extends SuperstructureSubsystem {

  private LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  private LauncherWrist m_launcherWrist = LauncherWrist.getInstance();
  private LauncherHold m_launcherHold = LauncherHold.getInstance();

  private boolean m_noteInLauncher = false;

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

    outputCommand.addCommands(new SetVelocitySubsystemState(m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState));
    outputCommand.addCommands(new SetPositionSubsystemState(m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState));
    outputCommand.addCommands(new SetVoltageSubsystemState(m_launcherHold, launcherDesiredState.launcherHoldState));
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = launcherDesiredState));

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

    if (m_desiredState == LauncherSuperstructureState.INTAKING && !m_launcherBeamBreak.get() && m_noteInLauncher == false) {
      new SequentialCommandGroup(
      new WaitCommand(.1),
      setSuperstructureState(LauncherSuperstructureState.OFF)
      .alongWith(IntakeSuperstructure.getInstance().setSuperstructureState(IntakeSuperstructureState.STOWED))).schedule();
      m_noteInLauncher = true;
    }

    if (!m_launcherBeamBreak.get() && !m_noteInLauncher) {
      m_noteInLauncher = true;
    } else if (m_launcherBeamBreak.get() && m_noteInLauncher) {
      m_noteInLauncher = false;
    }


    if (Constants.kInfoMode) {
      Logger.recordOutput(m_name + "/Superstructure/BeamBreak", m_launcherBeamBreak.get());
    }
    
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
    INTAKING(
        LauncherFlywheelState.OFF,
        LauncherWristState.LAUNCH,
        LauncherHoldState.INTAKING,
        "Intaking"),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      LauncherHoldState.OFF,
      "Transition"),
    SUBWOOFER(
      LauncherFlywheelState.RUNNING,
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
