package frc.lib.utilities;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public final class CtreUtils {

  /**
   *
   *
   * <h3>generateCanCoderConfig</h3>
   *
   * Generates and returns the configuration of the sensor.
   *
   * @return - The config of the sensor
   */
  public static CANcoderConfiguration generateCanCoderConfig() {
    CANcoderConfiguration sensorConfig = new CANcoderConfiguration();

    sensorConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

    return sensorConfig;
  }


  /**
   *
   *
   * <h3>checkCtreError</h3>
   *
   * Checks for a specified error.
   *
   * @param errorCode - Code of error
   * @param message - Desired message if error detected
   */
  // public static void checkCtreError(ErrorCode errorCode, String message) {
  //   if (RobotBase.isReal() && errorCode != ErrorCode.OK) {
  //     DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
  //   }
  // }
}
