package frc.robot.subsystems.launcher;


import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LauncherConstants;
import frc.robot.commands.superstructure.SetVelocitySubsystemState;
// import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.templates.VelocitySubsystem;

public class LauncherFlywheel extends VelocitySubsystem {

    private static LauncherFlywheel m_instance = null;

    private SimpleMotorFeedforward m_flywheelFeedForward;

    private DigitalInput m_beamBreak;

    public LauncherFlywheel(VelocitySubsystemConstants constants) {
        super(constants);

        m_beamBreak = new DigitalInput(0);

        m_flywheelFeedForward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    }

    public static LauncherFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherFlywheel(LauncherConstants.kLauncherFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        // setFeedforward(m_flywheelFeedForward.calculate(getVelocity()));


        // LauncherFlywheelState.FIELD_BASED_VELOCITY.setVelocity(calculateRPM());

        if (m_beamBreak.get()) {
            new SetVelocitySubsystemState(m_instance, LauncherFlywheelState.OFF, null, null).schedule();
        }
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putBoolean("Beam Break", m_beamBreak.get());
        Logger.recordOutput("Beam Break", m_beamBreak.get());
    }

    public double calculateRPM() {
        // double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);

        // if (distance > 0 && distance < LauncherConstants.kDistanceRPMMap.lastKey()) {
        //     double lowerRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.floorKey(distance));
        //     double upperRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.ceilingKey(distance));
        //     return lowerRPM + (distance - Math.floor(distance)) * (upperRPM - lowerRPM);
        // } else {
        //     return 0;
        // }
        return 0;
    }

    public enum LauncherFlywheelState implements VelocitySubsystemState {
        OFF(new double[] {0, 0}, "Off"),
        IDLE(new double[] {1000, -1000}, "Idle"),
        FAST(new double[] {2500, -2500}, "Fast"),
        TRANSITION(new double[] {0, 0}, "Transition"),
        FIELD_BASED_VELOCITY(new double[] {0, 0}, "Field Based Velocity"),
        RUNNING(new double[] {5000, -5000}, "Running");
    
        private double[] velocity;
        private String name;
    
        private LauncherFlywheelState(double[] velocity, String name) {
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
        public void setVelocity(double[] velocity) {
            this.velocity = velocity;
        }
    }
    
}
