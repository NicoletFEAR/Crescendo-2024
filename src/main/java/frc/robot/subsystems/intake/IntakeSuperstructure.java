package frc.robot.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetMultiMotorPositionSubsystemState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;
import frc.robot.subsystems.intake.ElevatorLift.ElevatorLiftState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class IntakeSuperstructure extends SuperstructureSubsystem {
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
    if (m_currentState == IntakeSuperstructureState.AUTO_INTAKING && timeOfFlightBlocked() && !isNoteInIntake) {
      RobotContainer.m_intakeWrist.setDesiredState(IntakeWristState.STOWED, true);
      RobotContainer.m_intakeFlywheel.setState(IntakeFlywheelState.OFF);
      RobotContainer.m_elevatorLift.setDesiredState(ElevatorLiftState.DOWN, true);
      isNoteInIntake = true;
    }
    
    if ( isNoteInIntake && !timeOfFlightBlocked()){
      isNoteInIntake = false;
    }

    if ( !isNoteInIntake && timeOfFlightBlocked()){
      isNoteInIntake = true;
    }

    if(RobotContainer.m_intakeFlywheel.getCurrentState() == IntakeFlywheelState.OFF && RobotContainer.m_intakeWrist.atSetpoint() && RobotContainer.m_elevatorLift.atSetpoint() && getCurrentState() != IntakeSuperstructureState.STOWED) {
      setCurrentState(IntakeSuperstructureState.STOWED);
    }

  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    if (intakeDesiredState == IntakeSuperstructureState.STOWED) {
      outputCommand.addCommands(new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState));
      outputCommand.addCommands(new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState));
      outputCommand.addCommands(new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState));
      outputCommand.addCommands(new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState));
    } else {
      outputCommand.addCommands(
        new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState)
          .alongWith(new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState)));
      outputCommand.addCommands(new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState));
      outputCommand.addCommands(new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState));
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
    INTAKING( // note should pass through here and be stoped in the launcher hold by the beam break logic in the launcher superstructure
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.INTAKING,
        "Intaking"),
    AUTO_INTAKING( // note should stop in take flyhweels by the tof logic in this class
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.DOWN,
        IntakeHoldState.OFF,
        "Auto Intaking"),
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
    public static final double kTOFNoteThreshold = 230;
  }
}
