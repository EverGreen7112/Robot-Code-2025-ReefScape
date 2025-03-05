package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
    
    /*private static LedStrip m_instance = new LedStrip();
    private AddressableLED m_led = new AddressableLED(0);

    private LedStrip(){
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);
        LEDPattern green = LEDPattern.gradient(GradientType.kContinuous, Color.kLightGreen, Color.kGreen, Color.kForestGreen);
        Distance kLedSpacing = Meters.of(1 / 120.0);
        LEDPattern scrollingGreen = green.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
        scrollingGreen.applyTo(ledBuffer);
        m_led.setLength(ledBuffer.getLength());
        m_led.setData(ledBuffer);

    }*/

    



}
