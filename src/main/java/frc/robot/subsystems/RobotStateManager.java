package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.templates.SuperstructureSubsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.superstructure.SetLEDState;
import frc.robot.commands.superstructure.SetVoltageSubsystemState;
import frc.robot.commands.waits.WaitForIntakeNote;
import frc.robot.commands.waits.WaitForLaunchNote;
import frc.robot.subsystems.LED.LEDState;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;
import frc.robot.subsystems.launcher.LauncherSuperstructure;
import frc.robot.subsystems.launcher.LauncherHold.LauncherHoldState;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;

public class RobotStateManager extends SuperstructureSubsystem {

    private static RobotStateManager m_instance = null;

    IntakeSuperstructure m_intakeSuperstructure = RobotContainer.m_intakeSuperstructure;

    LauncherSuperstructure m_launcherSuperstructure = RobotContainer.m_launcherSuperstructure;

    public RobotStateManager(SuperstructureState initialState, String name) {
        super(initialState, name);
    }

    public static RobotStateManager getInstance() {
        if (m_instance == null) {
            m_instance = new RobotStateManager(RobotState.STOWED, "Robot State Manager");
        }

        return m_instance;
    }

    @Override
    public SequentialCommandGroup setSuperstructureState(SuperstructureState desiredState) {
        RobotState robotDesiredState = (RobotState) desiredState;
            
        SequentialCommandGroup outputCommand = new SequentialCommandGroup();

        outputCommand.addCommands(new SetLEDState(LEDState.TEAL_ELEVATOR_LEDS).onlyIf((() -> m_currentState == RobotState.AMP && robotDesiredState == RobotState.TRAVEL)));

        outputCommand.addCommands(new InstantCommand(() -> m_desiredState = robotDesiredState).alongWith(
            new InstantCommand(() -> m_currentState = RobotState.TRANSITION)
        ));

        switch(robotDesiredState){
            case AMP:
                handleAmpCommand(robotDesiredState, outputCommand);
                break;
            case BEAM_BREAK_INTAKING:
                handleBeamBreakIntakingCommand(robotDesiredState, outputCommand);
                break;
            case AUTO_INTAKING:
                handleAutoIntakingCommand(robotDesiredState, outputCommand);
                break;
            case TOF_INTAKING:
                handleTOFIntakingCommand(robotDesiredState, outputCommand);
                break;
            case TRAVEL:
                handleTravelCommand(robotDesiredState, outputCommand);
                break;
            case CLIMB_PREPARE:
                handleClimbCommand(robotDesiredState, outputCommand);
                break;
            case CHIN_UP:
                handleClimbCommand(robotDesiredState, outputCommand);
                break;
            default:
                handleDefaultCommand(robotDesiredState, outputCommand);
                break;
        }

        outputCommand.addCommands(new InstantCommand(() -> m_currentState = robotDesiredState));

        return outputCommand;
    }

    private void handleDefaultCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState).alongWith(
                m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
            )
        );
    }

    private void handleTravelCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState).alongWith(
                m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
            ),
            new SetLEDState(LEDState.TEAL_STOW)
        );
    }

    private void handleBeamBreakIntakingCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
                ),
            new WaitForLaunchNote(),
            new SetVoltageSubsystemState(RobotContainer.m_launcherHold, LauncherHoldState.OFF),
            new SetLEDState(LEDState.BLUE_FLASHING, 1.0, LEDState.GREEN_STOW),
            m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.IDLE)
                )
        );
    }

    private void handleAutoIntakingCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
                ),
            new WaitForLaunchNote(),
            new SetLEDState(LEDState.BLUE_FLASHING, 1.0, LEDState.GREEN_STOW),
            m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.DOWNOFF)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.IDLE)
                )
        );
    }

    private void handleTOFIntakingCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
                ),
            new WaitForIntakeNote(),
            new SetLEDState(LEDState.BLUE_FLASHING, 1.0, LEDState.GREEN_STOW),
            m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL)
        );
    }

    private void handleAmpCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            // new WaitForIntakeNote()
            // .alongWith(
            //   m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.LAUNCH_TO_INTAKE),
            //   m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_TO_INTAKE))
            //     .unless(() -> m_intakeSuperstructure.getNoteInIntake() || !m_launcherSuperstructure.getNoteInLauncher()),
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState).alongWith(
                m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState),
                new SetLEDState(LEDState.GREEN_ELEVATOR_LEDS)
            ),
            new SetLEDState(LEDState.TEAL_STOW)
        );
    }

    private void handleClimbCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState).alongWith(
                m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState),
                new SetLEDState(LEDState.TEAL_ELEVATOR_LEDS)
            ),
            new SetLEDState(LEDState.TEAL_STOW)
        );
    }


    @Override
    public SuperstructureState getTransitionState() {
        return RobotState.TRANSITION;
    }

    @Override
    public void superstructurePeriodic() {
        SmartDashboard.putString("Robot Current State", m_currentState.getName());
        SmartDashboard.putString("Robot Desired State", m_desiredState.getName());
    }

    public enum RobotState implements SuperstructureState {
        TRANSITION(
            IntakeSuperstructureState.TRANSITION,
            LauncherSuperstructureState.TRANSITION),
        AUTO_START_SUBWOOFER(
            IntakeSuperstructureState.DOWNOFF,
            LauncherSuperstructureState.FIELD_BASED_PREP),
        AMP(
            IntakeSuperstructureState.AMP_PREPARE,
            LauncherSuperstructureState.STOWED),
        CLIMB_PREPARE(
            IntakeSuperstructureState.CLIMB_PREPARE,
            LauncherSuperstructureState.STOWED),
        BEAM_BREAK_INTAKING(
            IntakeSuperstructureState.INTAKING,
            LauncherSuperstructureState.INTAKE_TO_LAUNCH),
        TOF_INTAKING(
            IntakeSuperstructureState.TOF_INTAKING,
            LauncherSuperstructureState.STOWED),
        AUTO_INTAKING(
            IntakeSuperstructureState.INTAKING,
            LauncherSuperstructureState.INTAKE_TO_LAUNCH),
        TRAVEL(
            IntakeSuperstructureState.TRAVEL,
            LauncherSuperstructureState.STOWED),
        CHIN_UP(
            IntakeSuperstructureState.TRAVEL,
            LauncherSuperstructureState.STOWED),
        STOWED(
            IntakeSuperstructureState.STOWED,
            LauncherSuperstructureState.STOWED);

        public IntakeSuperstructureState intakeSuperstructureState;
        public LauncherSuperstructureState launcherSuperstructureState;

        private RobotState(IntakeSuperstructureState intakeSuperstructureState, LauncherSuperstructureState launcherSuperstructureState) {
            this.intakeSuperstructureState = intakeSuperstructureState;
            this.launcherSuperstructureState = launcherSuperstructureState;
        }

        @Override
        public String getName() {
            return name();
        }
    }
    
}
