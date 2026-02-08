package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
 //import frc.robot.RobotState;
//import frc.utils.StormSubsystem;

import static edu.wpi.first.units.Units.*;

    //public class Lights extends StormSubsystem {
      public class Lights extends SubsystemBase {
        // Alliance variables
        //private final RobotState robotState;
        Color RED_COLOR = Color.kRed;
        Color BLUE_COLOR = Color.kBlue;
        Color ORANGE_COLOR = new Color(255, 32, 0);
        Color PINK_COLOR = new Color(225,105,180);
        private final DigitalInput hopperHasBall = new DigitalInput(8);
        private final DigitalInput hopperIsFull = new DigitalInput(7);

        private AddressableLED addressableLED;
        private AddressableLEDBuffer addressableLEDBuffer;
        private AddressableLEDBufferView m_left;
        private AddressableLEDBufferView m_right;

        private boolean pulse = false;
        private double pulsePeriod = 5.0;

        public Lights() {
           // addressableLED = new AddressableLED(Constants.Lights.port);
            addressableLED = new AddressableLED(0);

           // addressableLEDBuffer = new AddressableLEDBuffer(Constants.Lights.ledLength);
            addressableLEDBuffer = new AddressableLEDBuffer(33);
            addressableLED.setLength(addressableLEDBuffer.getLength());
            addressableLED.start();
             setViews();

         //   robotState = RobotState.getInstance();
        }
        public boolean isObjectDetected() {
            // Returns true if the beam is broken, false otherwise
            return hopperHasBall.get();
        }
        public boolean isHopperFull() {
            // Returns true if the beam is broken, false otherwise
            return hopperIsFull.get();
        }
        

        @Override
        public void periodic() {
            super.periodic();
            if (isObjectDetected()){
                setSolid(BLUE_COLOR);
                System.out.println("Blue");// blue is empty
            }
            else if (isHopperFull()){
                setSolid(PINK_COLOR);
                System.out.println("Pink"); //pink means neither full or empty
            }
            else{
                setSolid(RED_COLOR);
                System.out.println("Red");// red is full
            }
            
            

            // Write the data to the LED strip
            addressableLED.setData(addressableLEDBuffer);
            
        }

        // Wrapper function to allow pulse effect to be applied to any pattern. Call this function rather
        // than calling pattern.applyTo()
        private <T extends LEDReader & LEDWriter> void patternApplyTo(LEDPattern basePattern, T view) {
            LEDPattern finalPattern = basePattern;

            if (pulse) {
                finalPattern = basePattern.breathe(Seconds.of(pulsePeriod));
            }

            finalPattern.applyTo(view);
        }

        public void setRainbow(boolean forward) {
            LEDPattern rainbow = LEDPattern.rainbow(255, 128);
            Distance kLedSpacing = Meters.of(1 / 120.0);
            LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.45), kLedSpacing);
            if (!forward)
                scrollingRainbow = scrollingRainbow.reversed();

            patternApplyTo(scrollingRainbow, m_left);
            patternApplyTo(scrollingRainbow, m_right);
        }

        public void setSolid(Color color) {
            LEDPattern pattern = LEDPattern.solid(color);
            patternApplyTo(pattern, addressableLEDBuffer);
        }

        public void setViews() {
            // Split led buffer into left and right views, so we can apply a different pattern to each strip
            // Right view is reversed because LED strips are wired in series
           // m_left = addressableLEDBuffer.createView(Constants.Lights.leftViewStart, Constants.Lights.leftViewEnd);
          //  m_right = addressableLEDBuffer.createView(Constants.Lights.rightViewStart, Constants.Lights.rightViewEnd).reversed();
            m_left = addressableLEDBuffer.createView(0,16);
            m_right = addressableLEDBuffer.createView(17,32).reversed();
        }
    }

