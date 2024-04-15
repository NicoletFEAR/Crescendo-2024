package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    
    // private static ArrayList<Double> wiperEffect = new ArrayList<>(Arrays.asList(
    //     0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0
    //   ));
    
    
      // Size has to be divisible by length of led
      private static ArrayList<Double> rainEffect = new ArrayList<>(Arrays.asList(
        0.25, 1.0, 0.25, 0.0
      ));
    
    
      // private static ArrayList<Double> cometEffect = new ArrayList<>(Arrays.asList(
      //   0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9
      // ));

    private static ArrayList<Double> wiperEffect = new ArrayList<>(Arrays.asList(
      0.1, 0.4, 0.8, 1.0, 1.0, 1.0, 1.0, 0.8, 0.4, 0.1
    ));
    
    
      
      private static LED m_instance = null;
    
      private static AddressableLED m_led;
    
      private static AddressableLEDBuffer m_ledBuffer;
    
      private static LEDState m_currentState = LEDState.RED;
    
      private static int m_rainbowFirstPixelHue = 0;
    
      private boolean hasEffect = false;
    
      private static double frameRunTime = 0;
    
      private static boolean flashOn = false;
    
      private static double pulseMultiplier = 1.0;
    
      private static boolean pulseIncreasing = false;

      private static double m_effectRunTime = -1;

      private static LEDState m_postTimerState = null;

      private static double launcherFrozenVel = 0;

      private static double elevatorFrozenPos = 0;

      // Robot States



      // private static boolean transition = false;
      // private static boolean intaking = false;
      private static boolean noteInRobot = false;
      // private static boolean ampPrepare = false;
      // private static boolean launching = false;
      // private static boolean stowed = false;
      
      public LED(AddressableLED led, int length) {
        m_led = led;
    
        wiperEffect = createLoopingEffect(wiperEffect, length);
        rainEffect = createLoopingEffect(rainEffect, length);
    
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(length);
    
        m_led.setData(m_ledBuffer);
        m_led.start();
      }
    
      public static LED getInstance() {
        if (m_instance == null) {
          m_instance = new  LED(new AddressableLED(LEDConstants.kLedStripPort), LEDConstants.kLedStripLength);
        }
    
        return m_instance;
      }
    
      public static void setState(LEDState desiredState) {
        m_currentState = desiredState;
        if (desiredState.ledRunnable == null || desiredState == LEDState.GREEN_STOW || desiredState == LEDState.TEAL_STOW) {
          setRGB(desiredState.red, desiredState.green, desiredState.blue);
        } else if (desiredState == LEDState.GREEN_LAUNCHER_LEDS || desiredState == LEDState.RED_LAUNCHER_LEDS) {
          launcherFrozenVel = RobotContainer.m_launcherFlywheel.getVelocity()[0];
        } else if (desiredState == LEDState.GREEN_ELEVATOR_LEDS || desiredState == LEDState.TEAL_ELEVATOR_LEDS) {
          elevatorFrozenPos = RobotContainer.m_elevatorLift.getPosition()[0];
        }
      }

      public static void setState(LEDState desiredState, double seconds, LEDState postTimerState) {
        setState(desiredState);
        m_effectRunTime = seconds;
        m_postTimerState = postTimerState;
      }
    
      public static void setRGB(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, r, g, b);
        }
    
        m_led.setData(m_ledBuffer);
      }
    
      public LEDState getCurrentState() {
        return m_currentState;
      }
    
      public static void runEffect(ArrayList<Double> effect, double frameDuration) {
        if (frameRunTime >= frameDuration) {
          frameRunTime = 0;
          for (int i = 0; i < effect.size(); i++) {
            m_ledBuffer.setRGB(i, (int) (m_currentState.red * effect.get(i)), (int) (m_currentState.green * effect.get(i)), (int) (m_currentState.blue * effect.get(i)));
          }
          m_led.setData(m_ledBuffer);
          effect.add(0, effect.remove(effect.size() - 1));
        } else {
          frameRunTime += 0.02;
        }
      }
    
      public static void flash(double frameDuration) {
        if (frameRunTime >= frameDuration) {
          frameRunTime = 0;
          if (flashOn) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
              m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            flashOn = false;
          } else {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
              m_ledBuffer.setRGB(i, m_currentState.red, m_currentState.green, m_currentState.blue);
            }
            flashOn = true;
          }
          m_led.setData(m_ledBuffer);
        } else {
          frameRunTime += 0.02;
        }
      }
    
      public static void pulse(double speed) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, (int) (m_currentState.red * pulseMultiplier), (int) (m_currentState.green * pulseMultiplier), (int) (m_currentState.blue * pulseMultiplier));
        }
        m_led.setData(m_ledBuffer);
        
        if (pulseIncreasing) {
          pulseMultiplier += speed; // Scale this to run faster/slower
          if (pulseMultiplier >= 1.0) {
            pulseMultiplier = 1.0;
            pulseIncreasing = false;
          }
        } else {
          pulseMultiplier -= speed;
          if (pulseMultiplier <= 0.0) {
              pulseMultiplier = 0.0;
              pulseIncreasing = true;
          }
      }
        
      }
    
    
      public static void runEffect(ArrayList<Double> effect) {
        for (int i = 0; i < effect.size(); i++) {
          m_ledBuffer.setRGB(i, (int) (m_currentState.red * effect.get(i)), (int) (m_currentState.green * effect.get(i)), (int) (m_currentState.blue * effect.get(i)));
        }
        m_led.setData(m_ledBuffer);
        effect.add(0, effect.remove(effect.size() - 1));
      }
    
      public static void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
      }
    
      @Override
      public void periodic() {
        noteInRobot = RobotContainer.m_launcherSuperstructure.getNoteInLauncher() ||
              RobotContainer.m_intakeSuperstructure.getNoteInIntake();

        if (m_currentState.ledRunnable != null) {
          m_currentState.ledRunnable.run();
          if (!hasEffect) hasEffect = true;
        } else if (hasEffect) {
          hasEffect = false;
          setState(m_currentState);
        } 

        

        if (m_effectRunTime > 0) {
          m_effectRunTime -= 0.02;
        } else if (m_effectRunTime != -1 && m_effectRunTime <= 0) {
          m_effectRunTime = -1;
          setState(m_postTimerState);
        }

        if (Constants.kInfoMode) {
          SmartDashboard.putString("LED/LED State", getCurrentState().name());
        }
        
      }

      private ArrayList<Double> createLoopingEffect(ArrayList<Double> initialList, int targetLength) {
        ArrayList<Double> outputList = new ArrayList<>();
    
        for (int i = 0; i <= targetLength - initialList.size(); i += initialList.size()) {
          outputList.addAll(i, initialList);
        }
    
        return outputList;
      } 
    
    
      // private ArrayList<Double> setToLength(ArrayList<Double> initialList, int targetLength) {
      //   ArrayList<Double> outputList = new ArrayList<>();
    
      //   outputList.addAll(initialList);
    
      //   for (int i = 0; i < targetLength - initialList.size(); i++) {
      //     outputList.add(0.0);
      //   }
    
      //   return outputList;
      // } 

      public static void setLedToLauncherVelocity() {
        if (noteInRobot && m_currentState != LEDState.GREEN_LAUNCHER_LEDS) {
          setState(LEDState.GREEN_LAUNCHER_LEDS);
        } else if (!noteInRobot && m_currentState != LEDState.RED_LAUNCHER_LEDS && RobotContainer.m_launcherFlywheel.getVelocity()[0] < 3000) {
          setState(LEDState.RED_LAUNCHER_LEDS);
        }

        double velPercent;

        double currentVel = RobotContainer.m_launcherFlywheel.getVelocity()[0];
        double intendedVel = RobotContainer.m_launcherFlywheel.getDesiredState().getVelocity()[0];



        if (currentVel < intendedVel) {
          velPercent = currentVel / intendedVel;
        } else {
          velPercent = (currentVel - intendedVel) / (launcherFrozenVel - intendedVel);
        }

        velPercent = MathUtil.clamp(velPercent, 0, 1);
        double ledAmount = (int) ((26) * velPercent);

        for (int i = 2; i < 27; i++) {
          if (i <= ledAmount) {
            m_ledBuffer.setRGB(i, m_currentState.red, m_currentState.green, m_currentState.blue);
            m_ledBuffer.setRGB(i + 26, m_currentState.red, m_currentState.green, m_currentState.blue);
          } else {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_ledBuffer.setRGB(i + 26, 0, 0, 0);
          }
        }
        
        m_led.setData(m_ledBuffer);
      }

      public static void setLedToElevatorPosition() {

        if (noteInRobot && m_currentState != LEDState.GREEN_ELEVATOR_LEDS) {
          setState(LEDState.GREEN_ELEVATOR_LEDS);
        } else if (!noteInRobot && m_currentState != LEDState.TEAL_ELEVATOR_LEDS) {
          setState(LEDState.TEAL_ELEVATOR_LEDS);
        }

        double posPercent;

        double currentPos = RobotContainer.m_launcherFlywheel.getVelocity()[0];
        double intendedPos = RobotContainer.m_launcherFlywheel.getDesiredState().getVelocity()[0];        

        if (currentPos < intendedPos) {
          posPercent = currentPos / intendedPos;
        } else {
          posPercent = (currentPos - intendedPos) / (elevatorFrozenPos - intendedPos);
        }

        posPercent = MathUtil.clamp(posPercent, 0, 1);

        if (posPercent >= .98) posPercent = 1;
        
        double ledAmount = (int) ((26) * posPercent);

        for (int i = 2; i < 27; i++) {
          if (i <= ledAmount) {
            m_ledBuffer.setRGB(i, m_currentState.red, m_currentState.green, m_currentState.blue);
            m_ledBuffer.setRGB(i + 26, m_currentState.red, m_currentState.green, m_currentState.blue);
          } else {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_ledBuffer.setRGB(i + 26, 0, 0, 0);
          }
        }
        
        m_led.setData(m_ledBuffer);
      }

      public static void setLEDToStow() {
        if (noteInRobot && m_currentState != LEDState.GREEN_STOW) {
          setState(LEDState.GREEN_STOW);
        } else if (!noteInRobot && m_currentState != LEDState.TEAL_STOW) {
          setState(LEDState.TEAL_STOW);
        }
      }
    
      public enum LEDState {
        RED(255, 0, 0, null),
        TEAL(0, 122, 133, null),
        RED_LAUNCHER_LEDS(255, 0, 0, LED::setLedToLauncherVelocity),
        GREEN_LAUNCHER_LEDS(0, 255, 0, LED::setLedToLauncherVelocity),
        GREEN_ELEVATOR_LEDS(0, 255, 0, LED::setLedToElevatorPosition),
        TEAL_ELEVATOR_LEDS(0, 122, 133, LED::setLedToElevatorPosition),
        GREEN_STOW(0, 255, 0, LED::setLEDToStow),
        TEAL_STOW(0, 122, 133, LED::setLEDToStow),
        GREEN_FLASHING(0, 255, 0, () -> LED.flash(0.03)),
        RED_FLASHING(255, 0, 0, () -> LED.flash(0.03)),
        BLUE_FLASHING(0, 0, 255, () -> LED.flash(0.03)),
        GREEN_WIPE(0, 255, 0, () -> LED.runEffect(wiperEffect));
        
        

    
        public int red;
        public int green;
        public int blue;
        public Runnable ledRunnable;
    
        private LEDState(int red, int green, int blue, Runnable ledRunnable) {
          this.red = red;
          this.green = green;
          this.blue = blue;
          this.ledRunnable = ledRunnable;
        }
      }
    
    public class LEDConstants {
        public static final int kLedStripPort = 1;
        public static final int kLedStripLength = 53;

        public static final int kLedDesiredStripLength = 50;
    }
}
