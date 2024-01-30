package frc.robot.subsystems.intake;


import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.templates.PositionSubsystem;

public class IntakeWrist extends PositionSubsystem {


    private static IntakeWrist m_instance = null;

    public IntakeWrist(PositionSubsystemConstants constants) {
        super(constants);
    }

    public static IntakeWrist getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeWrist(IntakeConstants.kIntakeWristConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        IntakeWristState.FIELD_BASED_PITCH.setPosition(calculatePitch());
        SmartDashboard.putNumber("Calculated shooter Pitch", m_currentState.getPosition());
    }

    @Override
    public void outputTelemetry() {}

    public double calculatePitch() {
        // double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);

        // if (distance > 0 && distance < IntakeConstants.kDistancePitchMap.lastKey()) {
        //     double lowerPitch = IntakeConstants.kDistancePitchMap.get(IntakeConstants.kDistancePitchMap.floorKey(distance));
        //     double upperPitch = IntakeConstants.kDistancePitchMap.get(IntakeConstants.kDistancePitchMap.ceilingKey(distance));
        //     return lowerPitch + (distance - Math.floor(distance)) * (upperPitch - lowerPitch);
        // } else {
        //     return 0;
        // }

        return 0;
    }

    public enum IntakeWristState implements PositionSubsystemState {
        DOWN(5, 0, "Down"),
        UP(0, 0, "Up"),
        AMP(3.5, 0, "Amp"),
        FIELD_BASED_PITCH(0, 0, "Field Based Pitch"),
        TRANSITION(0, 0, "Transition"),
        MANUAL(0, 0, "Manual");
    
        private double position;
        private double velocity;
        private String name;
    
        private IntakeWristState(double position, double velocity, String name) {
          this.position = position;
          this.velocity = velocity;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void setPosition(double position) {
            this.position = position;
        }
    }
    
}
