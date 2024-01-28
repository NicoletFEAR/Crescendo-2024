package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.superstructure.SetPositionSubsystemState;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.intake.IntakeWrist.IntakeWristState;
import frc.robot.subsystems.templates.SuperstructureSubsystem;

public class IntakeSuperstructure extends SuperstructureSubsystem {

  private IntakeFlywheel m_intakeFlywheel = IntakeFlywheel.getInstance();
  private IntakeWrist m_intakeWrist = IntakeWrist.getInstance();

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
        "Off"),
    IN(
        IntakeFlywheelState.IN,
        IntakeWristState.DOWN,
        "In"),
    OUT(
        IntakeFlywheelState.OUT,
        IntakeWristState.DOWN,
        "Out"),
    AMP(
        IntakeFlywheelState.AMP,
        IntakeWristState.AMP,
        "Amp"),
    TRANSITION(
      IntakeFlywheelState.OFF,
      IntakeWristState.TRANSITION,
      "Transition"
    );


    public IntakeFlywheelState intakeFlywheelState;
    public IntakeWristState intakeWristState;
    public String name;

    private IntakeSuperstructureState(
        IntakeFlywheelState intakeFlywheelState,
        IntakeWristState intakeWristState,
        String name) {
      this.intakeFlywheelState = intakeFlywheelState;
      this.intakeWristState = intakeWristState;
      this.name = name;
    }

    @Override
    public String getName() {
      return name;
    }
  }
}
