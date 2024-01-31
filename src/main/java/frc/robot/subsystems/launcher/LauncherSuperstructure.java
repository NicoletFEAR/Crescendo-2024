package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
// import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
// import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class LauncherSuperstructure extends SuperstructureSubsystem {

  private LauncherFlywheel m_launcherFlywheel = LauncherFlywheel.getInstance();
  private LauncherWrist m_launcherWrist = LauncherWrist.getInstance();
  // private LauncherHold m_launcherHold = LauncherHold.getInstance();

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

    outputCommand.addCommands(new SetVelocitySubsystemState(m_launcherFlywheel, launcherDesiredState.launcherFlywheelState, this, launcherDesiredState)
    .alongWith(new SetPositionSubsystemState(m_launcherWrist, launcherDesiredState.launcherWristState, this, launcherDesiredState)));
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
    //   setSuperstructureState(LauncherSuperstructureState.NOTE_IN_HOLD).schedule();;
    // }


    Logger.recordOutput(m_name + " Superstructure/BeamBreak", m_launcherBeamBreak.get());
  }

  public enum LauncherSuperstructureState implements SuperstructureState {
    OFF(
        LauncherFlywheelState.OFF,
        LauncherWristState.DOWN,
        // LauncherHoldState.OFF,
        "Off"),
    IDLE(
        LauncherFlywheelState.IDLE,
        LauncherWristState.DOWN,
        // LauncherHoldState.OFF,
        "Idle"),
    RUNNING(
        LauncherFlywheelState.RUNNING,
        LauncherWristState.UP,
        // LauncherHoldState.FEEDING,
        "Launch"),
    FAST(
        LauncherFlywheelState.FAST,
        LauncherWristState.DOWN,
        // LauncherHoldState.FEEDING,
        "Feeding"),
    OUTTAKE(
        LauncherFlywheelState.OFF,
        LauncherWristState.DOWN,
        // LauncherHoldState.OUTTAKING,
        "Outtaking"),
    TRANSITION(
      LauncherFlywheelState.TRANSITION,
      LauncherWristState.TRANSITION,
      // LauncherHoldState.TRANSITION,
      "Transition"
    );


    public LauncherFlywheelState launcherFlywheelState;
    public LauncherWristState launcherWristState;
    // public LauncherHoldState launcherHoldState;
    public String name;

    private LauncherSuperstructureState(
        LauncherFlywheelState launcherFlywheelState,
        LauncherWristState launcherWristState,
        // LauncherHoldState launcherHoldState,
        String name) {
      this.launcherFlywheelState = launcherFlywheelState;
      this.launcherWristState = launcherWristState;
      // this.launcherHoldState = launcherHoldState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }
}
