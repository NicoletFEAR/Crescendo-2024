package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.templates.SuperstructureSubsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.waits.WaitForIntakeNote;
import frc.robot.commands.waits.WaitForLaunchNote;
import frc.robot.subsystems.intake.IntakeSuperstructure;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;
import frc.robot.subsystems.launcher.LauncherSuperstructure;
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

    private void handleBeamBreakIntakingCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
                ),
            new WaitForLaunchNote(),
            m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)
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
            m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)
        );
    }

    private void handleTOFIntakingCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState)
                .alongWith(
                    m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
                ),
            new WaitForIntakeNote(),
            m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.TRAVEL)
        );
    }

    private void handleAmpCommand(RobotState robotDesiredState, SequentialCommandGroup outputCommand) {
        outputCommand.addCommands(
            new WaitForIntakeNote()
            .alongWith(
              m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.LAUNCH_TO_INTAKE),
              m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.LAUNCH_TO_INTAKE))
                .unless(() -> m_intakeSuperstructure.timeOfFlightBlocked() || !m_launcherSuperstructure.getNoteInLauncher()),
            m_intakeSuperstructure.setSuperstructureState(IntakeSuperstructureState.AMP_PREPARE).alongWith(
                m_launcherSuperstructure.setSuperstructureState(LauncherSuperstructureState.STOWED)
            )
        );
    }


    @Override
    public SuperstructureState getTransitionState() {
        return RobotState.TRANSITION;
    }

    @Override
    public void superstructurePeriodic() {}

    public enum RobotState implements SuperstructureState {
        TRANSITION(
            IntakeSuperstructureState.TRANSITION,
            LauncherSuperstructureState.TRANSITION,
            "Transition"
        ),
        AMP(
            IntakeSuperstructureState.AMP_PREPARE,
            LauncherSuperstructureState.STOWED,
            "Amp"
        ),
        BEAM_BREAK_INTAKING(
            IntakeSuperstructureState.INTAKING,
            LauncherSuperstructureState.INTAKE_TO_LAUNCH,
            "Intaking"
        ),
        TOF_INTAKING(
            IntakeSuperstructureState.INTAKING,
            LauncherSuperstructureState.STOWED,
            "TOF Intaking"
        ),
        AUTO_INTAKING(
            IntakeSuperstructureState.INTAKING,
            LauncherSuperstructureState.INTAKE_TO_LAUNCH,
            "Auto Intaking"
        ),
        TRAVEL(
            IntakeSuperstructureState.TRAVEL,
            LauncherSuperstructureState.STOWED,
            "Travel"
        ),
        STOWED(
            IntakeSuperstructureState.STOWED,
            LauncherSuperstructureState.STOWED,
            "Stowed"
        );

        public IntakeSuperstructureState intakeSuperstructureState;
        public LauncherSuperstructureState launcherSuperstructureState;
        public String name;

        private RobotState(IntakeSuperstructureState intakeSuperstructureState, LauncherSuperstructureState launcherSuperstructureState, String name) {
            this.intakeSuperstructureState = intakeSuperstructureState;
            this.launcherSuperstructureState = launcherSuperstructureState;
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }
    
}
