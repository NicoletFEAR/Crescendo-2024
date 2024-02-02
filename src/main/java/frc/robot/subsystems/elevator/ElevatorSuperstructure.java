package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.elevator.ElevatorLift.ElevatorLiftState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;
import frc.robot.subsystems.elevator.ElevatorWinch.ElevatorWinchState;;

public class ElevatorSuperstructure extends SuperstructureSubsystem {

  private ElevatorLift m_elevatorLift = ElevatorLift.getInstance();
  private ElevatorWinch m_elevatorWinch = ElevatorWinch.getInstance();

  private static ElevatorSuperstructure m_instance = null;

  public ElevatorSuperstructure(SuperstructureState initialState, String name) {
    super(initialState, name);
  }

  public static ElevatorSuperstructure getInstance() {
    if (m_instance == null) {
      m_instance = new ElevatorSuperstructure(ElevatorSuperstructureState.OFF, "Intake");
    }

    return m_instance;
  }


  @Override
  public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
    ElevatorSuperstructureState intakeDesiredState = (ElevatorSuperstructureState) desiredState;

    SequentialCommandGroup outputCommand = new SequentialCommandGroup();

    // outputCommand.addCommands(new SetVoltageSubsystemState(
    //   m_intakeFlywheel, intakeDesiredState.intakeFlywheelState)
    //     .alongWith(new SetPositionSubsystemState(
    //       m_intakeWrist, intakeDesiredState.intakeWristState, this, intakeDesiredState)));
    // outputCommand.addCommands(new InstantCommand(() -> m_currentState = intakeDesiredState));

    return outputCommand;
  }

  @Override 
  public SuperstructureState getTransitionState() {
    return ElevatorSuperstructureState.TRANSITION;
  }

  public enum ElevatorSuperstructureState implements SuperstructureState {
    OFF(
        ElevatorWinchState.HIGH,
        ElevatorLiftState.UP,
        "Off"),
    IN(
        ElevatorWinchState.HIGH,
        ElevatorLiftState.DOWN,
        "In"),
    OUT(
        ElevatorWinchState.HIGH,
        ElevatorLiftState.DOWN,
        "Out"),
    AMP(
        ElevatorWinchState.HIGH,
        ElevatorLiftState.AMP,
        "Amp"),
    TRANSITION(
      ElevatorWinchState.HIGH,
      ElevatorLiftState.TRANSITION,
      "Transition"
    );


    public ElevatorWinchState elevatorWinchState;
    public ElevatorLiftState elevatorLiftState;
    public String name;

    private ElevatorSuperstructureState(
        ElevatorWinchState elevatorWinchState,
        ElevatorLiftState elevatorLiftState,
        String name) {
      this.elevatorWinchState = elevatorWinchState;
      this.elevatorLiftState = elevatorLiftState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }
}