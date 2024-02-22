package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.superstructure.SetMultiMotorPositionSubsystemState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetSuperstructureState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
// import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.intake.ElevatorLift.ElevatorLiftState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class IntakeSuperstructure extends SuperstructureSubsystem {

  private ElevatorLift m_elevatorLift = ElevatorLift.getInstance();
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
      m_instance = new IntakeSuperstructure(IntakeSuperstructureState.STOWED, "Intake");
    }

    return m_instance;
  }

  public IntakeFlywheel getIntakeFlywheel(){
    return m_intakeFlywheel;
  }

  public boolean timeOfFlightBlocked(){
    if(m_intakeTOF.getRange() < IntakeConstants.kTOFNoteThreshold){
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
    // if (m_currentState == IntakeSuperstructureState.INTAKING && timeOfFlightBlocked()) {
    //   // new SetSuperstructureState(this, IntakeSuperstructureState.STOWED).schedule();
    //   m_intakeWrist.setDesiredState(IntakeWristState.STOWED, true);
    //   m_intakeFlywheel.setState(IntakeFlywheelState.OFF);
    //   m_elevatorLift.setDesiredState(ElevatorLiftState.DOWN, true);
    //   if (m_intakeWrist.atSetpoint() && m_elevatorLift.atSetpoint()) {
    //     setCurrentState(IntakeSuperstructureState.STOWED);
    //   }
    //   isNoteInIntake = true;
    // }
    // else if ( isNoteInIntake && !timeOfFlightBlocked()){
    //   isNoteInIntake = false;
    // }

    if (Constants.kInfoMode) {
      Logger.recordOutput("tof", m_intakeTOF.getRange());
      Logger.recordOutput("tofbool", timeOfFlightBlocked());
      Logger.recordOutput("isNoteInIntake", isNoteInIntake);
    }


  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    if (intakeDesiredState == IntakeSuperstructureState.STOWED) {
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
    STOWED(
        IntakeFlywheelState.OFF,
        IntakeWristState.STOWED,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Stowed"),
    TRAVEL(
        IntakeFlywheelState.OFF,
        IntakeWristState.TRAVEL,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Travel"),
    INTAKING(
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.INTAKING,
        "Intaking"),
    AMP_PREPARE(
        IntakeFlywheelState.OFF,
        IntakeWristState.AMP,
        ElevatorLiftState.AMP,
        IntakeHoldState.OFF,
        "Amp Prepare"),
    AMP_SHOOT(
      IntakeFlywheelState.AMP,
      IntakeWristState.AMP,
      ElevatorLiftState.AMP,
      IntakeHoldState.AMP,
      "Amp Prepare"),
    EJECT(
        IntakeFlywheelState.EJECTING,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.EJECTING,
        "Ejecting"),
    DOWNOFF(
        IntakeFlywheelState.OFF,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "down off"),
    TRANSITION(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRANSITION,
      ElevatorLiftState.TRANSITION,
      IntakeHoldState.OFF,
      "Transition"),
    LAUNCHING(
      IntakeFlywheelState.INTAKING,
      IntakeWristState.LAUNCHING,
      ElevatorLiftState.DOWN,
      IntakeHoldState.INTAKING,
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
    public static final int kLaunchBeamBreakID = 0;
    public static final int elevatorLimitSwitchID = 6;

    public static final double kTOFNoteThreshold = 260;
  }
}
