package frc.robot.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetLedState;
import frc.robot.commands.superstructure.SetMultiMotorPositionSubsystemState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.waits.WaitForIntakeNote;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeHold.IntakeHoldState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;
import frc.robot.subsystems.leds.LED.LEDState;
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
    // if (m_currentState == IntakeSuperstructureState.TELE_INTAKING && timeOfFlightBlocked() && !isNoteInIntake) {
    //   RobotContainer.m_intakeWrist.setDesiredState(IntakeWristState.STOWED, true);
    //   RobotContainer.m_intakeFlywheel.setState(IntakeFlywheelState.OFF);
    //   RobotContainer.m_elevatorLift.setDesiredState(ElevatorLiftState.DOWN, true);
    //   isNoteInIntake = true;
    // }
    
    if ( isNoteInIntake && !timeOfFlightBlocked()){
      isNoteInIntake = false;
    } if ( !isNoteInIntake && timeOfFlightBlocked()){
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

    if (intakeDesiredState == IntakeSuperstructureState.AMP_PREPARE) {
      outputCommand.addCommands(
        new WaitForIntakeNote()
          .raceWith(
            setSuperstructureState(IntakeSuperstructureState.LAUNCH_TO_INTAKE),
            RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_TO_INTAKE)),
        setSuperstructureState(IntakeSuperstructureState.STOWED)
          .andThen(RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED))
      );
    }

    if (intakeDesiredState == IntakeSuperstructureState.STOWED) {
      outputCommand.addCommands(
        new SetLedState(LEDState.GREEN_BLINKING),
        new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState),
        new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState),
        new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState),
        new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState),
        new SetLedState(LEDState.GREEN)
      );
    } else {
      outputCommand.addCommands(
        new SetLedState(LEDState.GREEN_BLINKING),
        new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState)
          .alongWith(new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState, this, intakeDesiredState)),
        new SetVoltageSubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState),
        new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState),
        new SetLedState(LEDState.GREEN)
      );
    }

    outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    if (intakeDesiredState == IntakeSuperstructureState.TELE_INTAKING) {
      outputCommand.addCommands(
        new SetLedState(LEDState.GREEN_BLINKING),
        new WaitForIntakeNote(),
        setSuperstructureState(IntakeSuperstructureState.STOWED),
        new SetLedState(LEDState.BLUE)
      );
    }

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
        ElevatorLiftState.STOWED,
        IntakeHoldState.OFF,
        "Stowed"),
    TRAVEL(
        IntakeFlywheelState.OFF,
        IntakeWristState.TRAVEL,
        ElevatorLiftState.STOWED,
        IntakeHoldState.OFF,
        "Travel"),
    AUTO_INTAKING( // note should pass through here and be stoped in the launcher hold by the beam break logic in the launcher superstructure
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
        IntakeHoldState.INTAKING,
        "Intaking"),
    TELE_INTAKING( // note should stop in take flyhweels by the tof logic in this class
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
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
        ElevatorLiftState.STOWED,
        IntakeHoldState.EJECTING,
        "Ejecting"),
    LAUNCH_TO_INTAKE(
        IntakeFlywheelState.AMP,
        IntakeWristState.STOWED,
        ElevatorLiftState.STOWED,
        IntakeHoldState.EJECTING,
        "Launch To Intake"),
    DOWNOFF(
        IntakeFlywheelState.OFF,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
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
      ElevatorLiftState.STOWED,
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
