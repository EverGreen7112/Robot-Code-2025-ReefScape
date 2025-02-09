package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverMotionMagicPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;

public class Elevator extends SubsystemBase {

    private static final boolean DEBUG_MODE = true;

    public enum ElevatorLevel{
        GROUND(0),
        L1(2),
        L2(23.5),
        L3(52.5),
        L4(99);

        public final double height;

        private ElevatorLevel(double height){
            this.height = height;
        }

    }

    private static Elevator m_instance = new Elevator();

    public EverMotorController m_motor;
    private EverPIDController m_pidController;
    private EverEncoder m_encoder;

    private DigitalInput m_topLS;
    private DigitalInput m_bottomLS;

    private Elevator(){

        EverTalonFX talon = new EverTalonFX(13);
        talon.setIdleMode(IdleMode.kCoast);
        talon.getControllerInstance().setNeutralMode(NeutralModeValue.Brake);

        EverMotionMagicPIDController talonPidController = new EverMotionMagicPIDController(talon, 90, 75);
        Slot0Configs a = new Slot0Configs();
        a.kD = 0;
        a.kG = 0.4;
        a.kI = 0;
        a.kP = 2.8;
        a.kV = 1/2.6;
        a.kS = 0.2;
        a.GravityType = GravityTypeValue.Elevator_Static;
        talonPidController.setPID(a);

        EverTalonFXInternalEncoder encoder = new EverTalonFXInternalEncoder(talon);
        encoder.setPosConversionFactor(1);
        
        m_topLS = new DigitalInput(2);
        m_bottomLS = new DigitalInput(1);

        m_motor = talon;
        m_pidController = talonPidController;
        m_encoder = encoder;
        
    }

    public static Elevator getInstance(){
        return m_instance;
    }

    public double getPose(){
        return m_encoder.getPos();
    }

    public void moveManually(double output){
        m_motor.set(output);
    }

    public void moveTo(double pos){
        m_pidController.activate(pos, ControlType.kPos);
    }

    public void moveToDesiredLevel(ElevatorLevel desiredLevel){
        moveTo(desiredLevel.height);
    }

    public boolean cantGoUp(){
        return m_topLS.get();
    }

    public boolean cantGoDown(){
        return m_bottomLS.get();
    }

    @Override
    public void periodic() {
        
        if(cantGoUp() && m_motor.get() > 0)
            m_motor.stop();

        if(cantGoDown() && m_motor.get() <= 0){
            m_motor.stop();
            m_encoder.setPos(0);
        }

        if(DEBUG_MODE)
            log();
    }  

    private void log(){
        SmartDashboard.putBoolean("topLs", m_topLS.get());
        SmartDashboard.putBoolean("bottomLs", m_bottomLS.get());
        SmartDashboard.putNumber("motor output", m_motor.get() * 12);
        SmartDashboard.putNumber("height",m_encoder.getPos());
    }

  
}
