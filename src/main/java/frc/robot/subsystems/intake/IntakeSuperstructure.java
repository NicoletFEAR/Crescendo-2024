package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetMultiMotorPositionSubsystemState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetSuperstructureState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.intake.MultiElevatorLift.MultiElevatorLiftState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class IntakeSuperstructure extends SuperstructureSubsystem {

  private MultiElevatorLift m_elevatorLift = MultiElevatorLift.getInstance();
  private IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  private IntakeWrist m_intakeWrist = IntakeWrist.getInstance();
  private IntakeHold m_intakeHold = IntakeHold.getInstance();
  private TimeOfFlight m_intakeTOF = new TimeOfFlight(0);
  private static boolean isNoteInIntake = false;

  private static IntakeSuperstructure m_instance = null;

  public IntakeSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);
  }

  public static IntakeSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSuperstructure(IntakeSuperstructureState.STOW, "Intake");
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
  }

  @Override
  public void superstructurePeriodic() {
    if (m_currentState == IntakeSuperstructureState.INTAKING && timeOfFlightBlocked()) {
      new SetSuperstructureState(this, IntakeSuperstructureState.STOW).schedule();
      isNoteInIntake = true;
    }
    else if ( isNoteInIntake && !timeOfFlightBlocked()){
      isNoteInIntake = false;
    }

    Logger.recordOutput("tof", m_intakeTOF.getRange());
    Logger.recordOutput("tofbool", timeOfFlightBlocked());
    Logger.recordOutput("isNoteInIntake", isNoteInIntake);

  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    if (intakeDesiredState == IntakeSuperstructureState.STOW) {
      outputCommand.addCommands(
        new SetPositionSubsystemState(m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState)
        .alongWith(new SetMultiMotorPositionSubsystemState(m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState))
        .alongWith(new SetVoltageSubsystemState(m_intakeFlywheel, intakeDesiredState.intakeFlywheelState))
        .alongWith(new SetVoltageSubsystemState(m_intakeHold, intakeDesiredState.intakeHoldState))
      );
    } else {
      outputCommand.addCommands(
        new SetPositionSubsystemState(m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState)
        .alongWith(new SetMultiMotorPositionSubsystemState(m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState))
        .andThen(new SetVoltageSubsystemState(m_intakeFlywheel, intakeDesiredState.intakeFlywheelState))
        .andThen(new SetVoltageSubsystemState(m_intakeHold, intakeDesiredState.intakeHoldState))
      );
    }
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    return outputCommand;
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return IntakeSuperstructureState.TRANSITION;
  }

  public enum IntakeSuperstructureState implements SuperstructureState {
    STOW(
        IntakeFlywheelState.OFF,
        IntakeWristState.UP,
        MultiElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Stow"),
    INTAKING(
        IntakeFlywheelState.IN,
        IntakeWristState.DOWN,
        MultiElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Intaking"),
    AMP_PREPARE(
        IntakeFlywheelState.OFF,
        IntakeWristState.AMP,
        MultiElevatorLiftState.AMP,
        IntakeHoldState.OFF,
        "Amp Prepare"),
    AMP_SHOOT(
      IntakeFlywheelState.AMP,
      IntakeWristState.AMP,
      MultiElevatorLiftState.AMP,
      IntakeHoldState.AMP,
      "Amp Prepare"),
    EJECT(
        IntakeFlywheelState.OUT,
        IntakeWristState.DOWN,
        MultiElevatorLiftState.DOWN,
        IntakeHoldState.OUT,
        "Eject"),
    DOWNOFF(
        IntakeFlywheelState.OFF,
        IntakeWristState.DOWN,
        MultiElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "down off"),
    TRANSITION(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRANSITION,
      MultiElevatorLiftState.TRANSITION,
      IntakeHoldState.OFF,
      "Transition"),
    LAUNCHING(
      IntakeFlywheelState.IN,
      IntakeWristState.LAUNCHING,
      MultiElevatorLiftState.DOWN,
      IntakeHoldState.IN,
      "Launching");

    public IntakeFlywheelState intakeFlywheelState;
    public IntakeWristState intakeWristState;
    public MultiElevatorLiftState elevatorLiftState;
    public IntakeHoldState intakeHoldState;
    public String name;

    private IntakeSuperstructureState(
        IntakeFlywheelState intakeFlywheelState,
        IntakeWristState intakeWristState,
        MultiElevatorLiftState elevatorLiftState,
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

    public static final double kTOFNoteTrheshold = 260;
  }
}