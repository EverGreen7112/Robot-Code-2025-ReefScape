package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;

public class Elevator extends SubsystemBase {

    private static final boolean DEBUG_MODE = true;

    public enum ElevatorLevel{
    
        L1(0),
        L2(0),
        L3(0),
        L4(0);

        public final double height;

        private ElevatorLevel(double height){
            this.height = height;
        }

    }

    private static Elevator m_instance = new Elevator();

    private EverMotorController m_motor;
    private EverPIDController m_pidController;
    private EverEncoder m_encoder;

    private DigitalInput m_topLS;
    private DigitalInput m_bottomLS;

    private double m_targetHeight;

    private Elevator(){

        EverTalonFX talon = new EverTalonFX(0); 
        talon.setInverted(false);
        talon.setIdleMode(EverTalonFX.IdleMode.kBrake);

        EverTalonFXPIDController talonPidController = new EverTalonFXPIDController(talon);
        Slot0Configs a = new Slot0Configs();
        a.kA = 0;
        a.kD = 0;
        a.kG = 0;
        a.kI = 0;
        a.kP = 0;
        a.kS = 0;
        a.kV = 0;
        talonPidController.setPID(a);

        EverTalonFXInternalEncoder encoder = new EverTalonFXInternalEncoder(talon);
        encoder.setPosConversionFactor(0);
        
        m_topLS = new DigitalInput(0); 
        m_bottomLS = new DigitalInput(0);

        m_motor = talon;
        m_pidController = talonPidController;
        m_encoder = encoder;
        
    }

    public static Elevator getInstance(){
        return m_instance;
    }

    public void moveManually(double output){
        m_motor.set(output);
    }

    public void moveToDesiredLevel(ElevatorLevel desiredLevel){
        switch(desiredLevel){ 
            case L1:
                m_targetHeight = ElevatorLevel.L1.height;
                break;
            case L2:
                m_targetHeight = ElevatorLevel.L2.height;
                break;
            case L3:
                m_targetHeight = ElevatorLevel.L3.height;
                break;
            case L4:
                m_targetHeight = ElevatorLevel.L4.height;
                break;
        }
        m_pidController.activate(m_targetHeight, EverPIDController.ControlType.kPos);
    }

    private boolean cantGoUp(){
        return m_topLS.get();
    }

    private boolean cantGoDown(){
        return m_bottomLS.get();
    }

    @Override
    public void periodic() {
        
        if(cantGoUp() && m_motor.get() > 0)
            m_motor.stop();

        if(cantGoDown() && m_motor.get() < 0)
            m_motor.stop();

        if(DEBUG_MODE)
            log();
    }  

    private void log(){
        SmartDashboard.putBoolean("topLs", m_topLS.get());
        SmartDashboard.putBoolean("bottomLs", m_topLS.get());
        SmartDashboard.putNumber("motor output", m_motor.get());
        SmartDashboard.putNumber("height",m_encoder.getPos());
    }


    
}
