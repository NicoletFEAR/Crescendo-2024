package frc.robot.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.templates.SuperstructureSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetMultiMotorPositionSubsystemState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.intake.ElevatorLift.ElevatorLiftState;

public class IntakeSuperstructure extends SuperstructureSubsystem {
  private TimeOfFlight m_intakeTOF = new TimeOfFlight(0);
  private static boolean isNoteInIntake = false;

  private static IntakeSuperstructure m_instance = null;

  private GenericEntry tofEntry;

  public IntakeSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);

    tofEntry = RobotContainer.mainTab.add("TOF Blocked", getNoteInIntake()).withPosition(0, 1).withSize(1, 4).getEntry();

    // SmartDashboard.putBoolean("Is note in intake", false);
  }

  public static IntakeSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSuperstructure(IntakeSuperstructureState.STOWED, "Intake");
    }

    return m_instance;
  }

  public boolean getNoteInIntake(){
    if(m_intakeTOF.getRange() < IntakeConstants.kTOFNoteThreshold){
      return true;
    }
    else{
      return false;
    }

    // return SmartDashboard.getBoolean("Is note in intake", false);
    // return false;
  }

  public void outputTelemetry() {
  }

  @Override
  public void superstructurePeriodic() {
    if (isNoteInIntake && !getNoteInIntake()){
      isNoteInIntake = false;
    } if (!isNoteInIntake && getNoteInIntake()){
      isNoteInIntake = true;
    }

    tofEntry.setBoolean(isNoteInIntake);

    if (Constants.kInfoMode) {
      SmartDashboard.putNumber(m_name + "/" + "TOF Range", m_intakeTOF.getRange());
    }

    

  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(
      new InstantCommand(() -> {m_desiredState = intakeDesiredState; m_currentState = IntakeSuperstructureState.TRANSITION;})
    );

    switch (intakeDesiredState) {
      default:
        handleDefaultCommand(intakeDesiredState, outputCommand);
        break;
    }

    outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    return outputCommand;
  }

  private void handleDefaultCommand(IntakeSuperstructureState intakeDesiredState, SequentialCommandGroup outputCommand) {
    outputCommand.addCommands(
      new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState)
        .alongWith(
          new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState),
          new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState),
          new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState)
        )
    );
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return IntakeSuperstructureState.TRANSITION;
  }

  public enum IntakeSuperstructureState implements SuperstructureState {
    STOWED(
      IntakeFlywheelState.OFF,
      IntakeWristState.STOWED,
      ElevatorLiftState.STOWED,
      IntakeHoldState.OFF),
  TRAVEL(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRAVEL,
      ElevatorLiftState.STOWED,
      IntakeHoldState.OFF),
  INTAKING( 
      IntakeFlywheelState.INTAKING,
      IntakeWristState.DOWN,
      ElevatorLiftState.STOWED,
      IntakeHoldState.INTAKING),
  TOF_INTAKING( 
      IntakeFlywheelState.SLOW_INTAKING,
      IntakeWristState.DOWN,
      ElevatorLiftState.STOWED,
      IntakeHoldState.INTAKING),
  BEAM_BREAK_INTAKING( 
      IntakeFlywheelState.INTAKING,
      IntakeWristState.DOWN,
      ElevatorLiftState.STOWED,
      IntakeHoldState.INTAKING),
  AMP_PREPARE(
      IntakeFlywheelState.OFF,
      IntakeWristState.AMP,
      ElevatorLiftState.AMP,
      IntakeHoldState.OFF),
  CLIMB_PREPARE(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRAVEL,
      ElevatorLiftState.CLIMB,
      IntakeHoldState.OFF),
  EJECT(
      IntakeFlywheelState.EJECTING,
      IntakeWristState.DOWN,
      ElevatorLiftState.STOWED,
      IntakeHoldState.EJECTING),
  LAUNCH_TO_INTAKE(
      IntakeFlywheelState.LAUNCH_TO_INTAKE,
      IntakeWristState.NOTE_TO_LAUNCHER,
      ElevatorLiftState.STOWED,
      IntakeHoldState.EJECTING),
  INTAKE_TO_LAUNCH(
      IntakeFlywheelState.INTAKE_TO_LAUNCH,
      IntakeWristState.DOWN,
      ElevatorLiftState.STOWED,
      IntakeHoldState.INTAKE_TO_LAUNCH),
  DOWNOFF(
      IntakeFlywheelState.OFF,
      IntakeWristState.DOWN,
      ElevatorLiftState.STOWED,
      IntakeHoldState.OFF),
  TRANSITION(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRANSITION,
      ElevatorLiftState.TRANSITION,
      IntakeHoldState.OFF),
  LAUNCHING(
      IntakeFlywheelState.INTAKING,
      IntakeWristState.LAUNCHING,
      ElevatorLiftState.STOWED,
      IntakeHoldState.INTAKING);

    public IntakeFlywheelState intakeFlywheelState;
    public IntakeWristState intakeWristState;
    public ElevatorLiftState elevatorLiftState;
    public IntakeHoldState intakeHoldState;

    private IntakeSuperstructureState(
        IntakeFlywheelState intakeFlywheelState,
        IntakeWristState intakeWristState,
        ElevatorLiftState elevatorLiftState,
        IntakeHoldState intakeHoldState
        ) {
      this.intakeFlywheelState = intakeFlywheelState;
      this.intakeWristState = intakeWristState;
      this.elevatorLiftState = elevatorLiftState;
      this.intakeHoldState = intakeHoldState;
    }

    @Override
    public String getName() {
      return name();
    }
  }

  public static final class IntakeConstants{
    public static final double kTOFNoteThreshold = 200;
  }
}
