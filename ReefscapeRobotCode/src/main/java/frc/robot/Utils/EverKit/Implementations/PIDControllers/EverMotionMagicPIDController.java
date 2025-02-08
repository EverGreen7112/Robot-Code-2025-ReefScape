package frc.robot.Utils.EverKit.Implementations.PIDControllers;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class EverMotionMagicPIDController extends EverPIDController implements Periodic{
    
    private TalonFX m_controller;
    private EverTalonFX m_everController;
    private MotionMagicVoltage m_posControl;
    private MotionMagicVelocityVoltage m_velControl;

    public EverMotionMagicPIDController(EverTalonFX controller, double cruiseVelocity, double acceleration){
        m_controller = controller.getControllerInstance();
        m_everController = controller;
        m_posControl = new MotionMagicVoltage(0).withSlot(0);
        m_velControl = new MotionMagicVelocityVoltage(0).withSlot(0);
        m_everController.setMotionMagicConfig((new MotionMagicConfigs()).withMotionMagicCruiseVelocity(cruiseVelocity).withMotionMagicAcceleration(acceleration));

        
    }

    @Override
    public void setPIDF(double kp, double ki, double kd, double kf) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kp; 
        slot0Configs.kI = ki;
        slot0Configs.kD = kd;
        slot0Configs.kS = kf;
        m_everController.setPidConfig(slot0Configs);     
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kp; 
        slot0Configs.kI = ki;
        slot0Configs.kD = kd;
        slot0Configs.kV = 0;
        m_everController.setPidConfig(slot0Configs);
    }

    public void setPID(Slot0Configs config){
        m_everController.setPidConfig(config);     
    }

    public void setMotionMagic(MotionMagicConfigs config){
        m_everController.setMotionMagicConfig(config);
    }

    @Override
    public void resetIAccum() {
        throw new UnsupportedOperationException("Unimplemented method 'resetIAccum'");
    }

    @Override
    public void activate(double setpoint, ControlType type) {
        switch (type) {
            case kPos:
                setpoint /= m_everController.getPosConversionFactor();
                m_controller.setControl(m_posControl.withPosition(setpoint));

            break;
            case kVel:
                setpoint /= m_everController.getVelConversionFactor();
                m_controller.setControl(m_velControl.withVelocity(setpoint));
                break;    
            default:
                break;
        }

    }

    @Override
    public void stop() {
        m_controller.stopMotor();
    }

    @Override
    public void periodic() {
        
    }
    
}
