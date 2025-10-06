package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Dispenser.Dispenser;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.RobotOperatorController;
import frc.robot.Utils.EverKit.Periodic;

public class LedStrip implements Periodic {

    private static final Distance LED_SPACING = Meters.of(1.0 / 120.0);

    private static LedStrip m_instance = new LedStrip();

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private LEDPattern m_ledPattern;
    private boolean m_turnClimberLedsOn;

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
        ERROR_SWERVE(
            LEDPattern.solid(Color.kRed)
        ),
        ERROR_ELEVATOR(
            LEDPattern.solid(Color.kCrimson)
        ),
        ERROR_DISPENSER(
            LEDPattern.solid(Color.kDeepPink)
        ),
        ERROR_CLIMBER(
            LEDPattern.solid(Color.kChocolate)
        ),
        ERROR_CAMS(
            LEDPattern.solid(Color.kBlue)
        ),
        ERROR_GYRO(
            LEDPattern.solid(Color.kPurple)
        ),
        ERROR_SWERVE_CANCODERS(
            LEDPattern.solid(Color.kYellow)
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

        m_led.start();
    }

    public static LedStrip getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        
        
        // //error leds    
        //  if(!DriverStation.isEnabled()){
        //     if(!Swerve.getInstance().areMotorControllersConnected() ){
        //         setLedPattern(LedPattern.ERROR_SWERVE);
        //     }
        //     else if(!Swerve.getInstance().areAbsEncodersConnected() ){
        //         setLedPattern(LedPattern.ERROR_SWERVE_CANCODERS);
        //     }
        //     else if(!Swerve.getInstance().isGyroConnected() ){
        //         setLedPattern(LedPattern.ERROR_GYRO);
        //     }
        //     else if(!Elevator.getInstance().areMotorControllersConnected() ){
        //         setLedPattern(LedPattern.ERROR_ELEVATOR);
        //     }
        //     else if(!Dispenser.getInstance().areMotorControllersConnected()){
        //         setLedPattern(LedPattern.ERROR_DISPENSER);
        //     }
        //     else if(!Climber.getInstance().areMotorControllersConnected()){
        //         setLedPattern(LedPattern.ERROR_CLIMBER);
        //     }
        //     else if(!SwerveLocalizer.getInstance().areCamsConnected() ){
        //         setLedPattern(LedPattern.ERROR_CAMS);
        //     }
        //     else{
        //         setLedPattern(LedPattern.DEFAULT_COLOR);
        //     }
        //     m_turnClimberLedsOn = false;
        // }
       
        // else if(Climber.getInstance().isCageLocked()){
        //     m_turnClimberLedsOn = true;
        // }
        // else if(SwerveAutoController.isRobotAligning){
        //     setLedPattern(LedPattern.ROBOT_ALIGNING);
        // }
        // else if(RobotOperatorController.getInstance().getLed()){
        //     setLedPattern(LedPattern.READY_FOR_CORAL);
        // }
        // else if(Dispenser.getInstance().isAtEntry() || Dispenser.getInstance().isAtExit()){
        //     setLedPattern(LedPattern.CORAL_IN_ROBOT);
        // }
        // else if(m_turnClimberLedsOn){
        //     setLedPattern(LedPattern.CAGE_LOCKED);
        // }
        // else {
        //     setLedPattern(LedPattern.DEFAULT_COLOR);
        // }

        //setLedPattern(LedPattern.CAGE_LOCKED);

    }

    public void setLedPattern(LedPattern pattern){
        m_ledPattern = pattern.pattern;
        m_ledPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public void initialize(){
        start(PeriodicTime.kRobotPeriodic);
    }
    
}
