package frc.robot.subsystems.intake;

import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.VoltageSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeFlywheel extends VoltageSubsystem {

    private static IntakeFlywheel m_instance = null;
  private TimeOfFlight m_intakeTOF;

    protected IntakeFlywheel(VoltageSubsystemConstants constants) {
        super(constants);
         m_intakeTOF = new TimeOfFlight(0);
        //TODO Auto-generated constructor stub
    }

    public boolean timeOfFlightBlocked(){
        if(m_intakeTOF.getRange() < 300){
          return true;
        }
        else{
          return false;
        }
      }

    public static IntakeFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeFlywheel(IntakeFlywheelConstants.kIntakeFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {}

    public enum IntakeFlywheelState implements VoltageSubsystemState {
        OFF(0, "Off"),
        IN(12, "In"),
        OUT(-8, "Out"),
        AMP(-6, "Amp"),
        TRANSITION(0, "Transition"),
        MANUAL(0, "Manual");
        
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

    

    public class IntakeFlywheelConstants {
        public static final SparkConstants kIntakeFlywheelLeaderConstants = new SparkConstants();

        static {
            kIntakeFlywheelLeaderConstants.kID = MotorConstants.kIntakeFlywheelID;
            kIntakeFlywheelLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeFlywheelLeaderConstants.kName = "Intake Flywheel";
            kIntakeFlywheelLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeFlywheelLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeFlywheelLeaderConstants.kCurrentLimit = 80;
            kIntakeFlywheelLeaderConstants.kInverted = false;
            kIntakeFlywheelLeaderConstants.kKp = 0.00001;
            kIntakeFlywheelLeaderConstants.kKi = 0.0;
            kIntakeFlywheelLeaderConstants.kKd = 0.0;
            kIntakeFlywheelLeaderConstants.kKff = 0.0001675;
        }

        public static final VoltageSubsystemConstants kIntakeFlywheelConstants =
            new VoltageSubsystemConstants();

        static {
            kIntakeFlywheelConstants.kSubsystemName = "Intake Flywheel";
            kIntakeFlywheelConstants.kSuperstructureName = "Intake";

            kIntakeFlywheelConstants.kSubsystemType = VoltageSubsystemType.INTAKE;

            kIntakeFlywheelConstants.kLeaderConstants = kIntakeFlywheelLeaderConstants;
           
            kIntakeFlywheelConstants.kInitialState = IntakeFlywheelState.OFF;
        }
    }

    
}