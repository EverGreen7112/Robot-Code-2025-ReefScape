package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.regex.Pattern;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.Swerve.AutoDrive.DriveToSelectedPoseCommand;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Dispenser.Dispenser;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.RobotOperatorController;
import frc.robot.Utils.EverKit.Periodic;

public class LedStrip extends SubsystemBase implements Periodic {

    public static final Distance LED_SPACING = Meters.of(1.0 / 120.0);

    private static LedStrip m_instance = new LedStrip();

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private LEDPattern m_ledPattern;

    private LedPattern m_ledState;

    public enum LedPattern{
        
        DEFAULT_COLOR(
            LEDPattern.solid(Color.kGreen)
        ),
        CORAL_IN_ROBOT(
            LEDPattern.solid(Color.kGreenYellow)
        ),
        READY_FOR_CORAL(
            LEDPattern.solid(Color.kYellow)
        ),
        ROBOT_ALIGNING(
            LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.2), Seconds.of(0.1))

        ),
        CAGE_LOCKED(
            LEDPattern.rainbow(255,255).scrollAtAbsoluteSpeed(MetersPerSecond.of(3),LED_SPACING)
        ),
        ERROR_MOTOR(
            LEDPattern.solid(Color.kRed).blink(Seconds.of(1), Seconds.of(1))
        ),
        ERROR_CAMS(
            LEDPattern.solid(Color.kBlue).blink(Seconds.of(5), Seconds.of(1))
        ),
        ERROR_DISPENSER(
            LEDPattern.solid(null)
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
        
        m_ledPattern = LedPattern.DEFAULT_COLOR.pattern;

        start(PeriodicTime.kRobotPeriodic);
        m_led.start();
    }

    public static LedStrip getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
            
        if(!DriverStation.isEnabled()){
            if(!Swerve.getInstance().areMotorControllersConnected() ){
                setLedPattern(LedPattern.ERROR_MOTOR);
            }
            else if(!SwerveLocalizer.getInstance().areCamsConnected() ){
                setLedPattern(LedPattern.ERROR_CAMS);
            }
            else if(!Dispenser.getInstance().areMotorControllersConnected()){
                setLedPattern(m_ledState);
            }
            else{
                setLedPattern(LedPattern.DEFAULT_COLOR);
            }
        }
       
        else if(Climber.getInstance().isCageLocked()){
            setLedPattern(LedPattern.CAGE_LOCKED);
        }
        else if(SwerveAutoController.isRobotAligning){
            setLedPattern(LedPattern.ROBOT_ALIGNING);
        }
        else if(RobotOperatorController.getInstance().getLed()){
            setLedPattern(LedPattern.READY_FOR_CORAL);

        }
       
        else if(Dispenser.getInstance().isAtEntry() || Dispenser.getInstance().isAtExit()){
            setLedPattern(LedPattern.CORAL_IN_ROBOT);
        }
        else {
            setLedPattern(LedPattern.DEFAULT_COLOR);
        }

        
        SmartDashboard.putBoolean("sa", false);


    }

    public void setLedPattern(LedPattern pattern){
        m_ledPattern = pattern.pattern;
        m_ledPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
    
}
