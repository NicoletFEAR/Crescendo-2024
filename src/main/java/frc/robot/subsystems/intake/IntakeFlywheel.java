package frc.robot.subsystems.intake;


import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.templates.VoltageSubsystem;

public class IntakeFlywheel extends VoltageSubsystem {

    private static IntakeFlywheel m_instance = null;

    protected IntakeFlywheel(VoltageSubsystemConstants constants) {
        super(constants);
        //TODO Auto-generated constructor stub
    }

    public static IntakeFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeFlywheel(IntakeConstants.kIntakeFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        // TODO Auto-generated method stub
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
    }

    public enum IntakeFlywheelState implements VoltageSubsystemState {
        OFF(0, "Off"),
        IN(12, "In"),
        OUT(-8, "Out"),
        AMP(-6, "Amp");
        // TRANSITION(0, "Transition");
        
        private double voltage;
        private String name;
    
        private IntakeFlywheelState(double voltage, String name) {
          this.voltage = voltage;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public double getVoltage() {
            return voltage;
        }

        // @Override
        // public void setVoltage(double voltage) {
        //     this.voltage = voltage;
        // }
    }
    
}