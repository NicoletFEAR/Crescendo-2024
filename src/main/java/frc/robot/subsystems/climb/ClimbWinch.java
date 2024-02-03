package frc.robot.subsystems.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.templates.MultiMotorPositionSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.MultiMotorPositionSubsystemConstants;

public class ClimbWinch extends MultiMotorPositionSubsystem {

    private static ClimbWinch m_instance = null;

    public ClimbWinch(MultiMotorPositionSubsystemConstants constants) {
        super(constants);
    }

    public static ClimbWinch getInstance() {
        if (m_instance == null) {
            m_instance = new ClimbWinch(ClimbConstants.kClimbWinchConstants);
        }

        return m_instance;
    }

    @Override
    public void manualControl() {}

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {}
    

    public enum ClimbWinchState implements MultiMotorPositionSubsystemState {
        DOWN(new double[] {0, 0}, new double[] {0, 0}, "Down"),
        UP(new double[] {45, 45}, new double[] {0, 0}, "Up"),
        TRANSITION(new double[] {0, 0}, new double[] {0, 0}, "Transition"),
        MANUAL(new double[] {0, 0}, new double[] {0, 0}, "Manual");
    
        private double[] position;
        private double[] velocity;
        private String name;
    
        private ClimbWinchState(double[] position, double[] velocity, String name) {
          this.position = position;
          this.velocity = velocity;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public double[] getVelocity() {
            return velocity;
        }

        @Override
        public void setVelocity(double velocity, int index) {
            this.velocity[index] = velocity;
        }

        @Override
        public double[] getPosition() {
            return position;
        }

        @Override
        public void setPosition(double position, int index) {
            this.position[index] = position;
        }
    }
}
