package frc.lib.utilities;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public final class RevUtils {

  /**
   * Sets the drive motor configuration
   *
   * @param motorController the motor controller to tune
   */
  public static void setDriveMotorConfig(CANSparkFlex motorController) {

    boolean redoSet = true;
    REVLibError error;
    int tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setFF(DriveConstants.drivekff);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setP(DriveConstants.drivekp);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setI(DriveConstants.driveki);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setD(DriveConstants.drivekd);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setOpenLoopRampRate(DriveConstants.driverampRate);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    

    while(redoSet){
      error = motorController.setSmartCurrentLimit(60); //80
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      error = motorController.setSecondaryCurrentLimit(60, 0); //80
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setClosedLoopRampRate(0.1);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    // while(redoSet){
    //   error = motorController.setOpenLoopRampRate(0.0);
    //   if(error == REVLibError.kOk){
    //     redoSet = false;
    //   }
    //   tries++;
    //   if(tries >= 10){
    //     redoSet = false;
    //   }
    // }
    // redoSet = true;
    // tries = 0;
    

    while(redoSet){
      error = motorController.disableVoltageCompensation();
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    

    while(redoSet){
      error = motorController.enableSoftLimit(SoftLimitDirection.kForward, false);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    

    while(redoSet){
      error = motorController.enableSoftLimit(SoftLimitDirection.kReverse, false);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    // try {
    //   Thread.sleep(200);
    // } catch (Exception e) {}
    // motorController.burnFlash();
  }

  /**
   * Sets the turn motor configuration
   *
   * @param motorController the motor controller to tune
   */
  public static void setTurnMotorConfig(CANSparkMax motorController) {
    boolean redoSet = true;
    REVLibError error;
    int tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setFF(DriveConstants.turnkff);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setP(DriveConstants.turnkp);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setI(DriveConstants.turnki);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.getPIDController().setD(DriveConstants.turnkd);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;
    
    while(redoSet){
      error = motorController.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65534);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    while(redoSet){
      error = motorController.setSmartCurrentLimit(40, 25);
      if(error == REVLibError.kOk){
        redoSet = false;
      }
      tries++;
      if(tries >= 10){
        redoSet = false;
      }
    }
    redoSet = true;
    tries = 0;

    // try {
    //   Thread.sleep(200);
    // } catch (Exception e) {}
    // motorController.burnFlash();
  }

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
