package frc.robot.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetMultiMotorPositionSubsystemState;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.waits.WaitForIntakeNote;
import frc.robot.commands.waits.WaitForLaunchNote;
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

    SmartDashboard.putBoolean("Is note in intake", false);
  }

  public static IntakeSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new IntakeSuperstructure(IntakeSuperstructureState.STOWED, "Intake");
    }

    return m_instance;
  }

  public boolean timeOfFlightBlocked(){
    // if(m_intakeTOF.getRange() < IntakeConstants.kTOFNoteThreshold){
    //   return true;
    // }
    // else{
    //   return false;
    // }

    return SmartDashboard.getBoolean("Is note in intake", false);
  }

  public void outputTelemetry() {
  }

  @Override
  public void superstructurePeriodic() {
    if (isNoteInIntake && !timeOfFlightBlocked()){
      isNoteInIntake = false;
    } if (!isNoteInIntake && timeOfFlightBlocked()){
      isNoteInIntake = true;
    }

    // SmartDashboard.putBoolean("TOF Blocked", timeOfFlightBlocked());
  }

  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    IntakeSuperstructureState intakeDesiredState = (IntakeSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    outputCommand.addCommands(new InstantCommand(() -> m_desiredState = intakeDesiredState));
    outputCommand.addCommands(new InstantCommand(() -> m_currentState = IntakeSuperstructureState.TRANSITION));

    if (intakeDesiredState == IntakeSuperstructureState.STOWED || intakeDesiredState == IntakeSuperstructureState.TRAVEL || intakeDesiredState == IntakeSuperstructureState.INTAKE_TO_LAUNCH) {
      if (intakeDesiredState == IntakeSuperstructureState.STOWED) {
        outputCommand.addCommands(RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED));
      }
      outputCommand.addCommands(
        new SetVelocitySubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState),
        new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState),
        new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState),
        new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState)
      );
    } else if (intakeDesiredState == IntakeSuperstructureState.AMP_PREPARE) {
      outputCommand.addCommands(
        new WaitForIntakeNote()
          .alongWith(
            setSuperstructureState(IntakeSuperstructureState.LAUNCH_TO_INTAKE),
            RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_TO_INTAKE)).unless(() -> this.timeOfFlightBlocked() || !RobotContainer.m_launcherSuperstructure.getNoteInLauncher()),
        new SetVelocitySubsystemState(RobotContainer.m_intakeFlywheel, IntakeFlywheelState.OFF),
        new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState)
          .alongWith(RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED), 
            new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState)),
        new SetVelocitySubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState),
        new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState)
      );
    } else if (intakeDesiredState == IntakeSuperstructureState.FAST_BEAM_BREAK_INTAKING) {
        outputCommand.addCommands(
          new ParallelCommandGroup(
            new SetVelocitySubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState),
            new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState),
            new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState),
            new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState),
            RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.INTAKE_TO_LAUNCH)
          )
        );
    }

     else {
        if (intakeDesiredState == IntakeSuperstructureState.BEAM_BREAK_INTAKING) {
          outputCommand.addCommands(
            new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState)
            .alongWith(new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState))
            .alongWith(RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.INTAKE_TO_LAUNCH))
        );
      } else {
          outputCommand.addCommands(
          new SetPositionSubsystemState(RobotContainer.m_intakeWrist, intakeDesiredState.intakeWristState)
          .alongWith(new SetMultiMotorPositionSubsystemState(RobotContainer.m_elevatorLift, intakeDesiredState.elevatorLiftState))
        );
      }
      outputCommand.addCommands(
        new SetVelocitySubsystemState(RobotContainer.m_intakeFlywheel, intakeDesiredState.intakeFlywheelState),
        new SetVoltageSubsystemState(RobotContainer.m_intakeHold, intakeDesiredState.intakeHoldState)
      );
    }

    outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    if (intakeDesiredState == IntakeSuperstructureState.BEAM_BREAK_INTAKING || intakeDesiredState == IntakeSuperstructureState.FAST_BEAM_BREAK_INTAKING) {
      outputCommand.addCommands(
        new WaitForLaunchNote(),
        new WaitCommand(.1),
        setSuperstructureState(IntakeSuperstructureState.TRAVEL)
        .alongWith(RobotContainer.m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED))
      );
    }

    if (intakeDesiredState == IntakeSuperstructureState.TOF_INTAKING) {
      outputCommand.addCommands(
        new WaitForIntakeNote(),
        setSuperstructureState(IntakeSuperstructureState.TRAVEL)
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
        "Auto Intaking"),
    BEAM_BREAK_INTAKING( 
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
        IntakeHoldState.INTAKING,
        "Beam Break Intaking"),
    FAST_BEAM_BREAK_INTAKING( 
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
        IntakeHoldState.INTAKING,
        "Beam Break Intaking"),
    TOF_INTAKING( // note should stop in take flyhweels by the tof logic in this class
        IntakeFlywheelState.INTAKING,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
        IntakeHoldState.OFF,
        "Tof Intaking"),
    AMP_PREPARE(
        IntakeFlywheelState.OFF,
        IntakeWristState.AMP,
        ElevatorLiftState.AMP,
        IntakeHoldState.OFF,
        "Amp Prepare"),
    CLIMB_PREPARE(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRAVEL,
      ElevatorLiftState.CLIMB,
      IntakeHoldState.OFF,
      "Climb Prepare"),
    EJECT(
        IntakeFlywheelState.EJECTING,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
        IntakeHoldState.EJECTING,
        "Ejecting"),
    LAUNCH_TO_INTAKE(
        IntakeFlywheelState.LAUNCH_TO_INTAKE,
        IntakeWristState.NOTE_TO_LAUNCHER,
        ElevatorLiftState.STOWED,
        IntakeHoldState.EJECTING,
        "Launch To Intake"),
    INTAKE_TO_LAUNCH_PREPARE(
        IntakeFlywheelState.OFF,
        IntakeWristState.NOTE_TO_LAUNCHER,
        ElevatorLiftState.STOWED,
        IntakeHoldState.OFF,
        "Intake To Launch"),
    INTAKE_TO_LAUNCH(
        IntakeFlywheelState.INTAKE_TO_LAUNCH,
        IntakeWristState.DOWN,
        ElevatorLiftState.STOWED,
        IntakeHoldState.INTAKE_TO_LAUNCH,
        "Intake To Launch"),
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
