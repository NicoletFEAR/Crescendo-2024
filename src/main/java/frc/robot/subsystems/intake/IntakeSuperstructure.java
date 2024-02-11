package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.ElevatorLift.ElevatorLiftState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class IntakeSuperstructure extends SuperstructureSubsystem {

  private ElevatorLift m_elevatorLift = ElevatorLift.getInstance();
  private IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  private IntakeWrist m_intakeWrist = IntakeWrist.getInstance();
  private IntakeHold m_intakeHold = IntakeHold.getInstance();
  private TimeOfFlight m_intakeTOF = new TimeOfFlight(0);

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

  public boolean timeOfFlightBlocked(){
    if(m_intakeTOF.getRange() < IntakeConstants.kTOFNoteTrheshold){
      return true;
    }
    else{
      return false;
    }
  }

  public void outputTelemetry() {
    Logger.recordOutput("Time Of Flight", m_intakeTOF.getRange());
    Logger.recordOutput("Time Of Flight Boolean", timeOfFlightBlocked());
  }



  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(
      new SetVoltageSubsystemState(m_intakeFlywheel, intakeDesiredState.intakeFlywheelState)
      .alongWith(new SetPositionSubsystemState(m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState))
      .alongWith(new SetPositionSubsystemState(m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState))
      .alongWith(new SetVoltageSubsystemState(m_intakeHold, intakeDesiredState.intakeHoldState))
    );
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    return outputCommand;
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return IntakeSuperstructureState.TRANSITION;
  }
  
  
  @Override
  public void superstructurePeriodic() {}

  public enum IntakeSuperstructureState implements SuperstructureState {
    OFF(
        IntakeFlywheelState.OFF,
        IntakeWristState.UP,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Off"),
    IN(
        IntakeFlywheelState.IN,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "In"),
    OUT(
        IntakeFlywheelState.OUT,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Out"),
    AMP(
        IntakeFlywheelState.AMP,
        IntakeWristState.AMP,
        ElevatorLiftState.AMP,
        IntakeHoldState.OFF,
        "Amp"),
    TRANSITION(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRANSITION,
      ElevatorLiftState.TRANSITION,
      IntakeHoldState.OFF,
      "Transition"),
    LAUNCHING(
      IntakeFlywheelState.IN,
      IntakeWristState.DOWN,
      ElevatorLiftState.DOWN,
      IntakeHoldState.IN,
      "Launching");

    public IntakeFlywheelState intakeFlywheelState;
    public IntakeWristState intakeWristState;
    public ElevatorLiftState elevatorLiftState;
    public IntakeHoldState intakeHoldState;
    public String name;

    private IntakeSuperstructureState(
        IntakeFlywheelState intakeFlywheelState,
        IntakeWristState intakeWristState,
        ElevatorLiftState elevatorLiftState,
        IntakeHoldState intakeHoldState,
        String name) {
      this.intakeFlywheelState = intakeFlywheelState;
      this.intakeWristState = intakeWristState;
      this.elevatorLiftState = elevatorLiftState;
      this.intakeHoldState = intakeHoldState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }

  public static final class IntakeConstants{
    public static final int kWristCANCoderID = 5;
    public static final int kLaunchBeamBreakID = 0;
    public static final int elevatorLimitSwitchID = 6;

    public static final double kTOFNoteTrheshold = 300;
  }
}