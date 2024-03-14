package frc.robot.subsystems.launcher;

import java.util.HashMap;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.utilities.PolarCoordinate;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetLEDState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.waits.WaitForLaunchNote;
import frc.robot.commands.waits.WaitForNoLaunchNote;
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

    RobotContainer.mainTab.add("Note In Launcher", m_noteInLauncher).withPosition(1, 1).withSize(1, 4);

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


    if (launcherDesiredState == LauncherSuperstructureState.STOWED) {
      outputCommand.addCommands(
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState))
          .alongWith(new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState))
      );
    } else if (launcherDesiredState == LauncherSuperstructureState.RUNNING) {
      outputCommand.addCommands(
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.THRU_LAUNCHER_INTAKING));
        new WaitCommand(.02);
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.OFF);
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)
      );
    } else if (launcherDesiredState == LauncherSuperstructureState.THRU_INTAKE_INTAKING) {
      outputCommand.addCommands(
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState)),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState)
      );
    } else if (launcherDesiredState == LauncherSuperstructureState.IDLE) {
      outputCommand.addCommands(
        new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState)
      );
    } else {
      outputCommand.addCommands(
        new SetVelocitySubsystemState(RobotContainer.m_launcherFlywheel, launcherDesiredState.launcherFlywheelState)
          .alongWith(new SetPositionSubsystemState(RobotContainer.m_launcherWrist, launcherDesiredState.launcherWristState)),
        new WaitCommand(.02),
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, launcherDesiredState.launcherHoldState),
        new WaitForNoLaunchNote(),
        new WaitCommand(.04)
      );
    }

    if (launcherDesiredState == LauncherSuperstructureState.KEEP_NOTE_IN_LAUNCH) {
      outputCommand.addCommands(
        new WaitForLaunchNote().andThen(
        new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.OFF))
      );
    }

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
    if (!m_launcherBeamBreak.get() && !m_noteInLauncher) {
      m_noteInLauncher = true;
    } if (m_launcherBeamBreak.get() && m_noteInLauncher) {
      m_noteInLauncher = false;
    }

    // m_noteInLauncher = SmartDashboard.getBoolean("note in launcher", m_noteInLauncher);
    if (Constants.kInfoMode) {
      SmartDashboard.putBoolean(m_name + "/" + "launcher beam break", m_launcherBeamBreak.get());
    }

    
  }

  public enum LauncherSuperstructureState implements SuperstructureState {
    STOWED(
        LauncherFlywheelState.OFF,
        LauncherWristState.UP,
        LauncherHoldState.OFF,
        "Stowed"),
    IDLE(
        LauncherFlywheelState.OFF,
        LauncherWristState.SUBWOOFER,
        LauncherHoldState.OFF,
        "Idle"),
    RUNNING( // arbitrary testing speed
        LauncherFlywheelState.RUNNING,
        LauncherWristState.UP,
        LauncherHoldState.LAUNCHING,
        "Running"),
    THRU_INTAKE_INTAKING(
        LauncherFlywheelState.OFF,
        LauncherWristState.UP,
        LauncherHoldState.THRU_INTAKE_INTAKING,
        "Thru Intake Intaking"),
    KEEP_NOTE_IN_LAUNCH(
        LauncherFlywheelState.OFF,
        LauncherWristState.UP,
        LauncherHoldState.THRU_INTAKE_INTAKING,
        "Keep Note In Launch"),
    INTAKE_TO_LAUNCH(
        LauncherFlywheelState.OFF,
        LauncherWristState.UP,
        LauncherHoldState.THRU_INTAKE_INTAKING,
        "Intake To Launch"),
    LAUNCH_TO_INTAKE(
        LauncherFlywheelState.INTAKING,
        LauncherWristState.UP,
        LauncherHoldState.THRU_LAUNCHER_INTAKING,
        "Launch To Intake"),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      LauncherHoldState.OFF,
      "Transition"),
    PODIUM( // use when against base of podium
      LauncherFlywheelState.PODIUM,
      LauncherWristState.PODIUM,
      LauncherHoldState.LAUNCHING,
      "Podium"
    ),
    WING_NOTE_1( 
      LauncherFlywheelState.WING_NOTE_1,
      LauncherWristState.WING_NOTE_1,
      LauncherHoldState.LAUNCHING,
      "Wing Note 1"
    ),
    WING_NOTE_2( 
      LauncherFlywheelState.WING_NOTE_2,
      LauncherWristState.WING_NOTE_2,
      LauncherHoldState.LAUNCHING,
      "Wing Note 2"
    ),
    WING_NOTE_3( 
      LauncherFlywheelState.WING_NOTE_3,
      LauncherWristState.WING_NOTE_3,
      LauncherHoldState.LAUNCHING,
      "Wing Note 3"
    ),
    LAUNCH_POS_1(
      LauncherFlywheelState.LAUNCH_POS_1,
      LauncherWristState.LAUNCH_POS_1,
      LauncherHoldState.LAUNCHING,
      "Launch Pos 1"
    ),
    LAUNCH_POS_2(
      LauncherFlywheelState.LAUNCH_POS_2,
      LauncherWristState.LAUNCH_POS_2,
      LauncherHoldState.LAUNCHING,
      "Launch Pos 2"
    ),
    LAUNCH_POS_3(
      LauncherFlywheelState.LAUNCH_POS_3,
      LauncherWristState.LAUNCH_POS_3,
      LauncherHoldState.LAUNCHING,
      "Launch Pos 3"
    ),
    POOP_POS_1( 
      LauncherFlywheelState.POOP_POS_1,
      LauncherWristState.POOP_POS_1,
      LauncherHoldState.LAUNCHING,
      "Poop Pos 1"
    ),
    POOP_POS_2( 
      LauncherFlywheelState.POOP_POS_2,
      LauncherWristState.POOP_POS_2,
      LauncherHoldState.LAUNCHING,
      "Poop Pos 2"
    ),
    POOP_POS_3(
      LauncherFlywheelState.POOP_POS_3,
      LauncherWristState.POOP_POS_3,
      LauncherHoldState.LAUNCHING,
      "Poop Pos 3"
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
    PASS( // use when against base of speaker
      LauncherFlywheelState.SUBWOOFER,
      LauncherWristState.PASS,
      LauncherHoldState.LAUNCHING,
      "Pass"
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
