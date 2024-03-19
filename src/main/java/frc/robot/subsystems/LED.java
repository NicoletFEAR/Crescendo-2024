package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSuperstructure.IntakeSuperstructureState;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherSuperstructureState;

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
    
      private static LEDState m_currentState = LEDState.OFF;
    
      private static int m_rainbowFirstPixelHue = 0;
    
      private boolean hasEffect = false;
    
      private static double frameRunTime = 0;
    
      private static boolean flashOn = false;
    
      private static double pulseMultiplier = 1.0;
    
      private static boolean pulseIncreasing = false;

      private static double m_effectRunTime = -1;

      // Robot States

      private static boolean transition = false;
      private static boolean intaking = false;
      private static boolean noteInRobot = false;
      private static boolean ampPrepare = false;
      private static boolean launching = false;
      private static boolean stowed = false;
      
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
        if (desiredState.ledRunnable == null) {
          setRGB(desiredState.red, desiredState.green, desiredState.blue);
        }
      }

      public static void setState(LEDState desiredState, double seconds) {
        setState(desiredState);
        m_effectRunTime = seconds;
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
        if (m_currentState.ledRunnable != null) {
          m_currentState.ledRunnable.run();
          if (!hasEffect) hasEffect = true;
        } else if (hasEffect) {
          hasEffect = false;
          setState(m_currentState);
        } 

        updateBasedOnMechs();

        if (m_effectRunTime > 0) {
          m_effectRunTime -= 0.02;
        } else if (m_effectRunTime != -1 && m_effectRunTime <= 0) {
          m_effectRunTime = -1;
        }

        if (Constants.kInfoMode) {
          SmartDashboard.putString("LED/LED State", getCurrentState().name());
        }
        
      }

      public void updateBasedOnMechs() {
        LEDState desiredState = null;

        transition = RobotContainer.m_intakeSuperstructure.getCurrentState() == IntakeSuperstructureState.TRANSITION ||
                      RobotContainer.m_launcherSuperstructure.getCurrentState() == LauncherSuperstructureState.TRANSITION;
        noteInRobot = RobotContainer.m_launcherSuperstructure.getNoteInLauncher() || RobotContainer.m_intakeSuperstructure.timeOfFlightBlocked();
        ampPrepare = RobotContainer.m_intakeSuperstructure.getDesiredState() == IntakeSuperstructureState.AMP_PREPARE;
        launching = RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.SUBWOOFER ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.PODIUM ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.POOP_POS_1 ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.POOP_POS_2 ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.WING_NOTE_1 ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.WING_NOTE_2 ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.WING_NOTE_3 ||
                      RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.PASS;
        intaking = RobotContainer.m_intakeSuperstructure.getDesiredState() == IntakeSuperstructureState.TOF_INTAKING ||
                    RobotContainer.m_intakeSuperstructure.getDesiredState() == IntakeSuperstructureState.BEAM_BREAK_INTAKING ||
                    RobotContainer.m_intakeSuperstructure.getDesiredState() == IntakeSuperstructureState.FAST_BEAM_BREAK_INTAKING;
        stowed = RobotContainer.m_launcherSuperstructure.getDesiredState() == LauncherSuperstructureState.STOWED ||
                  RobotContainer.m_intakeSuperstructure.getDesiredState() == IntakeSuperstructureState.STOWED ||
                  RobotContainer.m_intakeSuperstructure.getDesiredState() == IntakeSuperstructureState.TRAVEL;


        if (DriverStation.isEnabled() && m_effectRunTime == -1) {
          if (transition) {
            if (noteInRobot) {
              if (launching) {
                desiredState = LEDState.GREEN_REVVING;
              } else if (ampPrepare || stowed) {
                desiredState = LEDState.GREEN_BLINKING;
              } else if (intaking) {
                desiredState = LEDState.RED_BLINKING;
              }
            } else {
              if (ampPrepare || launching) {
                desiredState = LEDState.RED_BLINKING;
              } else if (intaking || stowed) {
                desiredState = LEDState.TEAL_BLINKING;
              }
            }
          } else {
            if (noteInRobot) {
              if (ampPrepare || stowed) {
                desiredState = LEDState.GREEN;
              } else if (intaking) {
                setState(LEDState.GREEN_WIPE, 1);
              }
            } else {
              if (ampPrepare || launching) {
                desiredState = LEDState.RED;
              } else if (stowed || intaking) {
                desiredState = LEDState.TEAL;
              }
            }
          }
        }

        if (desiredState != null && m_currentState != desiredState) {
          setState(desiredState);
        }
      }
    
      private ArrayList<Double> createLoopingEffect(ArrayList<Double> initialList, int targetLength) {
        ArrayList<Double> outputList = new ArrayList<>();
    
        for (int i = 0; i <= targetLength - initialList.size(); i += initialList.size()) {
          outputList.addAll(i, initialList);
        }
    
        return outputList;
      } 
    
    
      private ArrayList<Double> setToLength(ArrayList<Double> initialList, int targetLength) {
        ArrayList<Double> outputList = new ArrayList<>();
    
        outputList.addAll(initialList);
    
        for (int i = 0; i < targetLength - initialList.size(); i++) {
          outputList.add(0.0);
        }
    
        return outputList;
      } 

      public static void setLedToLauncherVelocity() {
        double velPercent = RobotContainer.m_launcherFlywheel.getVelocity()[0] / RobotContainer.m_launcherFlywheel.getDesiredState().getVelocity()[0];

        velPercent = MathUtil.clamp(velPercent, 0, 1);

        if (velPercent >= 1 && !RobotContainer.m_launcherSuperstructure.getNoteInLauncher()) {
          setState(LEDState.GREEN_FLASHING, 1.25);
        }

        double ledAmount = (int) ((LEDConstants.kLedStripLength / 2) * velPercent);

        for (int i = 0; i < LEDConstants.kLedStripLength / 2; i++) {
          if (i <= ledAmount) {
            m_ledBuffer.setRGB(i, m_currentState.red, m_currentState.green, m_currentState.blue);
            m_ledBuffer.setRGB(i + LEDConstants.kLedStripLength / 2, m_currentState.red, m_currentState.green, m_currentState.blue);
          } else {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_ledBuffer.setRGB(i + LEDConstants.kLedStripLength / 2, 0, 0, 0);
          }
        }
        
        m_led.setData(m_ledBuffer);
      }
    
      public enum LEDState {
        OFF(0, 0, 0, "Off", null),
        BLUE(0, 0, 255, "Blue", null),
        RED(255, 0, 0, "Red", null),
        GREEN(0, 255, 0, "Green", null),
        ORANGE(255, 102, 25, "Orange", null),
        YELLOW(255,255,25, "Yellow", null),
        WHITE(255, 255, 255, "White", null),

        ORANGE_BLINKING(255, 102, 25, "Orange Blinking", () -> LED.flash(0.1)),
        GREEN_BLINKING(0, 255, 0, "Green Blinking", () -> LED.flash(0.1)),
        RED_BLINKING(255, 0, 0, "Red Blinking", () -> LED.flash(0.1)),
        TEAL_BLINKING(0, 122, 133, "Teal Blinking", () -> LED.flash(0.1)),

        GREEN_REVVING(0, 255, 0, "Revving", LED::setLedToLauncherVelocity),
        BLUE_REVVING(0, 0, 255, "Blue Revving", LED::setLedToLauncherVelocity),

        GREEN_FLASHING(0, 255, 0, "Green Flashing", () -> LED.flash(0.03)),
        BLUE_FLASHING(0, 0, 255, "Blue Flashing", () -> LED.flash(0.04)),

        GREEN_WIPE(0, 255, 0, "Blue Wipe", () -> LED.runEffect(wiperEffect)),
        BLUE_WIPE(0, 0, 255, "Blue Wipe", () -> LED.runEffect(wiperEffect, 0.04)),
        ORANGE_WIPE(255, 25, 0, "Orange Wipe", () -> LED.runEffect(wiperEffect)),


        YELLOW_WIPE(255, 255, 25, "Yellow Wipe", () -> LED.runEffect(wiperEffect, 0.02)),

        // RAINBOW(0, 0, 0, "Rainbow",  LED::rainbow),
        TEAL(0, 122, 133, "Teal", null);
        // TEAL_RAIN(0, 122, 133, "Teal Rain", () ->  LED.runEffect(rainEffect, .2)),
        // TEAL_PULSE(0, 122, 133, "Teal Pulse", () ->  LED.pulse(.01)),
        // BLUE_FLASH(0, 0, 255, "Blue Flash", () ->  LED.flash(.2)),
        // ORANGE_FLASH(255, 179, 0, "Orange Flash", () ->  LED.flash(.2)),
    
        // TEAL_COMET(0, 122, 133, "Teal Comet", () ->  LED.runEffect(cometEffect, .2));
    
        public int red;
        public int green;
        public int blue;
        public Runnable ledRunnable;
        public String name;
    
        private LEDState(int red, int green, int blue, String name, Runnable ledRunnable) {
          this.red = red;
          this.green = green;
          this.blue = blue;
          this.ledRunnable = ledRunnable;
          this.name = name;
        }
      }
    
    public class LEDConstants {
        public static final int kLedStripPort = 0;
        public static final int kLedStripLength = 50;
    }
}
