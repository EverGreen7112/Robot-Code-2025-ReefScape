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

public class Elevator extends SubsystemBase {

    private static final boolean DEBUG_MODE = true;

    public enum ElevatorLevel{
        CLOSED(0, 0.2),
        L1(2, 0.2),
        L2(23, 0.2),
        L3(52, 0.2),
        L4(99, 0.1); //98, 0.08

        public final double height;
        public final double dispenseSpeed;

        private ElevatorLevel(double height, double dispenseSpeed){
            this.height = height;
            this.dispenseSpeed = dispenseSpeed;
        }

    }

    private static Elevator m_instance = new Elevator();

    private ElevatorLevel m_targetLevel;
    private EverMotorController m_motor;
    private EverPIDController m_pidController;
    private EverEncoder m_encoder;

    private DigitalInput m_topLS;
    private DigitalInput m_bottomLS;

    private Elevator(){
        m_targetLevel = ElevatorLevel.CLOSED;

        EverTalonFX talon = new EverTalonFX(13);
        talon.setIdleMode(IdleMode.kCoast);
        talon.getControllerInstance().setNeutralMode(NeutralModeValue.Brake);

        EverMotionMagicPIDController talonPidController = new EverMotionMagicPIDController(talon, 200, 90);
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

    public void moveToDesiredLevel(ElevatorLevel desiredLevel){
        m_targetLevel = desiredLevel;
        m_pidController.activate(desiredLevel.height, ControlType.kPos);
    }

    public ElevatorLevel getTargetLevel(){
        return m_targetLevel;
    }

    public boolean cantGoUp(){
        return m_topLS.get();
    }

    public boolean cantGoDown(){
        return m_bottomLS.get();
    }

    @Override
    public void periodic() {
        if(DEBUG_MODE)
            log();

        if(cantGoUp() && m_motor.get() > 0)
            m_motor.stop();

        if(cantGoDown() && m_motor.get() < 0){
            m_motor.stop();
            m_encoder.setPos(0);
        }

    }  

    private void log(){
        SmartDashboard.putBoolean("topLs", m_topLS.get());
        SmartDashboard.putBoolean("bottomLs", m_bottomLS.get());
        SmartDashboard.putNumber("motor output", m_motor.get());
        SmartDashboard.putNumber("height",m_encoder.getPos());
    }

  
}
