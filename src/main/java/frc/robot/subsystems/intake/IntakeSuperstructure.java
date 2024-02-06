package frc.robot.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.ElevatorLift.ElevatorLiftState;
import frc.robot.subsystems.intake.Hold.HoldState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;


public class IntakeSuperstructure extends SuperstructureSubsystem {

  private ElevatorLift m_ElevatorLift = ElevatorLift.getInstance();
  private IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  private IntakeWrist m_intakeWrist = IntakeWrist.getInstance();
  private TimeOfFlight m_tof = new TimeOfFlight(0);

  private static IntakeSuperstructure m_instance = null;

  public IntakeSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);
  }

  public static IntakeSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSuperstructure(IntakeSuperstructureState.OFF, "Intake");
    }

    return m_instance;
  }

  public IntakeFlywheel getIntakeFlywheel(){
    return m_intakeFlywheel;
  }

  public double getTOF(){
    return m_tof.getRange();
  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(new SetVoltageSubsystemState(
      m_intakeFlywheel, intakeDesiredState.intakeFlywheelState)
        .alongWith(new SetPositionSubsystemState(
          m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState)));
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    return outputCommand;
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return IntakeSuperstructureState.TRANSITION;
  }

  public enum IntakeSuperstructureState implements SuperstructureState {
    OFF(
        IntakeFlywheelState.OFF,
        IntakeWristState.UP,
        ElevatorLiftState.DOWN,
        HoldState.OFF,
        "Off"),
    IN(
        IntakeFlywheelState.IN,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        HoldState.OFF,
        "In"),
    OUT(
        IntakeFlywheelState.OUT,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        HoldState.OFF,
        "Out"),
    AMP(
        IntakeFlywheelState.AMP,
        IntakeWristState.AMP,
        ElevatorLiftState.AMP,
        HoldState.OFF,
        "Amp"),
    TRANSITION(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRANSITION,
      ElevatorLiftState.TRANSITION,
      HoldState.OFF,
      "Transition"),
    LAUNCHING(
      IntakeFlywheelState.IN,
      IntakeWristState.DOWN,
      ElevatorLiftState.DOWN,
      HoldState.IN,
      "Launching");

    public IntakeFlywheelState intakeFlywheelState;
    public IntakeWristState intakeWristState;
    public ElevatorLiftState elevatorLiftState;
    public HoldState holdState;
    public String name;

    private IntakeSuperstructureState(
        IntakeFlywheelState intakeFlywheelState,
        IntakeWristState intakeWristState,
        ElevatorLiftState elevatorLiftState,
        HoldState holdState,
        String name) {
      this.intakeFlywheelState = intakeFlywheelState;
      this.intakeWristState = intakeWristState;
      this.elevatorLiftState = elevatorLiftState;
      this.holdState = holdState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }

  @Override
  public void superstructurePeriodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'superstructurePeriodic'");
  }
}