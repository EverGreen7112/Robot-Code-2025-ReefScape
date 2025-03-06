package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.regex.Pattern;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import frc.robot.Subsystems.Climber.Climber;
=======
>>>>>>> origin/Leds
import frc.robot.Utils.EverKit.Periodic;

public class LedStrip extends SubsystemBase implements Periodic {

    public static final Distance LED_SPACING = Meters.of(1.0 / 120.0);

    private static LedStrip m_instance = new LedStrip();

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private LEDPattern m_ledPattern;

    private LedPattern m_ledState;

    public enum LedPattern{
        
        DEFULT_COLOR(
            LEDPattern.solid(Color.kGreen)
        ),
<<<<<<< HEAD
        CORAL_IN_ROBOT(
            LEDPattern.solid(Color.kGreenYellow)
        ),
        READY_FOR_CORAL(
            LEDPattern.solid(Color.kYellow)
        ),
        ROBOT_ALIGNING(
            LEDPattern.solid(Color.kGreen).blink(Second.of(1),Second.of(1))
        ),
        CAGE_LOCKED(
            LEDPattern.rainbow(255,255).scrollAtAbsoluteSpeed(MetersPerSecond.of(3),LED_SPACING)
=======
        FEEDER_ALIGNMENT_FINISHED(
            LEDPattern.solid(Color.kYellow).blink(Second.of(1), Second.of(1))
        ),
        BRANCH_ALIGNMENT_FINISHED(
            LEDPattern.solid(Color.kPurple)
        ),
        CORAL_DISPENSED(
            LEDPattern.solid(Color.kGreen).blink(Second.of(1),Second.of(1))
        ),
        CAGE_LOCKED(
            // crazy 
            LEDPattern.rainbow(255,255).scrollAtAbsoluteSpeed(MetersPerSecond.of(3),LED_SPACING)
                
>>>>>>> origin/Leds
        );

        public final LEDPattern pattern;

        private LedPattern(LEDPattern Pattern){
            this.pattern = Pattern;
        }
    }
    

    private LedStrip(){
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());
        
        m_ledPattern = LedPattern.DEFULT_COLOR.pattern;

        start(PeriodicTime.kRobotPeriodic);
        m_led.start();
    }

    public static LedStrip getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
<<<<<<< HEAD
        if(Climber.getInstance().get)

=======
>>>>>>> origin/Leds
        m_ledPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);

    }

    public void setLedPattern(LedPattern pattern){
        m_ledPattern = pattern.pattern;
    }
    
}
