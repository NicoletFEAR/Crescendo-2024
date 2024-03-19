package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.templates.SuperstructureSubsystem;
import frc.robot.RobotContainer;
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
            
        SequentialCommandGroup output = new SequentialCommandGroup();

        output.addCommands(new InstantCommand(() -> m_desiredState = robotDesiredState).alongWith(
            new InstantCommand(() -> m_currentState = RobotState.TRANSITION)
        ));

        output.addCommands(
            m_intakeSuperstructure.setSuperstructureState(robotDesiredState.intakeSuperstructureState).alongWith(
                m_launcherSuperstructure.setSuperstructureState(robotDesiredState.launcherSuperstructureState)
            )
        );

        output.addCommands(new InstantCommand(() -> m_currentState = robotDesiredState));

        return output;
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
