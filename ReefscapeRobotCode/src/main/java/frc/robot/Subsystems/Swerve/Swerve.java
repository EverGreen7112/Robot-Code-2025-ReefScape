package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverGyro;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Gyros.EverNavX;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

/**
NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
 * */
public class Swerve extends SubsystemBase implements SwerveConsts{
    final boolean DEBUG_MODE = true;

    public static Swerve m_instance = new Swerve();

    public SwerveModule[] m_modules;
    private AHRS m_gyro;

    private Vector2d m_velocity;
    private double m_angularVelocity;
    private boolean m_isGyroOriented;

    private Swerve() {
        SwerveConsts.config();
        m_modules = SwerveConsts.MODULES;
        m_velocity = new Vector2d(0, 0);
        m_angularVelocity = 0;
        m_isGyroOriented = true;
        m_gyro = new AHRS(NavXComType.kMXP_SPI);
    }

    /**
     * @return the only instance of the swerve
     */
    public static Swerve getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        if(DEBUG_MODE)
            log();
        updateModules();
    }

    private void updateModules(){
        //convert to m/s
        double angularVel = (m_angularVelocity / 360.0) * SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;

        if (m_velocity.mag() == 0 && angularVel == 0) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].stopModule();
            }
        }

        // convert to gyro oriented
        if(m_isGyroOriented)
            m_velocity.rotate(Math.toRadians(getGyroOrientedAngle() * SwerveConsts.GYRO_DIRECTION));
        
        // calculate rotation vectors
        Vector2d[] rotVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(Funcs.convertFromStandardAxesToWpilibs(SwerveConsts.modulesPositions[i]));
            rotVecs[i].rotate(Math.toRadians(90 * SwerveConsts.GYRO_DIRECTION));
            rotVecs[i].normalise();
            rotVecs[i].mul(angularVel);
        }

        Vector2d[] sumVectors = new Vector2d[m_modules.length];
        for (int i = 0; i < sumVectors.length; i++) {
            sumVectors[i] = new Vector2d(m_velocity);
            sumVectors[i].add(rotVecs[i]);
            m_modules[i].setState(sumVectors[i]);
        }
    }

    public double getGyroOrientedAngle(){
        return m_gyro.getYaw() * SwerveConsts.GYRO_DIRECTION;
    }

    public SwerveModule[] getModules(){
        return m_modules;
    }

    /**
     * see math on pdf document for more information
     * NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
     * 
     * @param velocity    - robot's target velocity(m/s)
     * @param isGyroOriented - true for origin of the gyro relative driving
     *                         false for robot relative driving
     * @param angularVelocity = robot's target angular velocity(deg/s) 
     */
    public void drive(Vector2d velocity, boolean isGyroOriented, double angularVelocity) {
        m_velocity = velocity;
        m_angularVelocity = angularVelocity;
        m_isGyroOriented = isGyroOriented;

    }

    public void driveByAngularVelocity(double angularVelocity){
        m_angularVelocity = angularVelocity;
    }

    public void driveByVelocity(Vector2d velocity, boolean isGyroOriented){
        m_velocity = velocity;
        m_isGyroOriented = isGyroOriented;
    }

    public void driveRobotOrientedBySpeeds(ChassisSpeeds speeds){
        drive(new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), false,
            Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    /**
     * @return robot's angular velocity in NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
     * degrees/sec 
     */
    public double getAngularVelocity() {
        double angularVelocity = 0;
        
        for (int i = 0; i < m_modules.length; i++) {
            Vector2d moduleRotationVector = new Vector2d(Funcs.convertFromStandardAxesToWpilibs(SwerveConsts.modulesPositions[i]));
            moduleRotationVector.normalise();
            moduleRotationVector.rotate(Math.toRadians(90 * SwerveConsts.GYRO_DIRECTION));

            Vector2d moduleVelocity = m_modules[i].getVelocity();

            angularVelocity += moduleVelocity.dot(moduleRotationVector);
        }
        
        // at this point the angular velocity is in m/s
        angularVelocity /= (double)SwerveConsts.modulesPositions.length;

        // converts angularVelocity to degrees/s
        angularVelocity /= SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;  // rotations / sec
        angularVelocity *= 360;  // degrees / sec

        return angularVelocity;
    }

    /**
     * @return robot's velocity in NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
     */
    public Vector2d getRobotOrientedVelocity(){
        Vector2d vel = new Vector2d();
        for(int i = 0; i < m_modules.length; i++){
            vel.add(m_modules[i].getVelocity());
        }
        vel.mul(1.0 / m_modules.length);
        return vel;
    }

    public ChassisSpeeds getRobotOrientedSpeeds() {
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.omegaRadiansPerSecond = Math.toRadians(getAngularVelocity());
        speeds.vxMetersPerSecond = getRobotOrientedVelocity().x;
        speeds.vyMetersPerSecond = getRobotOrientedVelocity().y;
        return speeds;
    }

    public Vector2d getGyroOrientedVelocity(){
        Vector2d vel = getRobotOrientedVelocity();
        vel.rotate(Math.toRadians(getGyroOrientedAngle() * GYRO_DIRECTION));
        return vel;
    }
    
    public void resetModulesDistance(){
        for(int i = 0; i < m_modules.length; i++){
            m_modules[i].resetDistance();
        }
    }

    /**
     * DO NOT use it in the middle of the game unless you have to.
     * every action that depends on the localization might not work 
     */
    public void resetGyro(){
        m_gyro.reset();
    }

   
    public SwerveModulePosition[] getModulesPositions() {
        return new SwerveModulePosition[]{
            m_modules[0].getPosition(), 
            m_modules[1].getPosition(), 
            m_modules[2].getPosition(), 
            m_modules[3].getPosition()};
    }

    public Rotation2d getGyroRotation2d(){
        return new Rotation2d(Math.toRadians(getGyroOrientedAngle()));
    }

    public void testModule(int moduleIdx, double targetAngle, double targetSpeed){
        m_modules[moduleIdx].setState(targetSpeed, targetAngle);
    }

    private void log(){
        // SmartDashboard.putNumber("TL", m_modules[0].getAngle());
        // SmartDashboard.putNumber("TR", m_modules[1].getAngle());
        // SmartDashboard.putNumber("DL", m_modules[2].getAngle());
        // SmartDashboard.putNumber("DR", m_modules[3].getAngle());

        SmartDashboard.putString("velocity", getRobotOrientedVelocity().toString());
        SmartDashboard.putNumber("angular velocity", getAngularVelocity());
        SmartDashboard.putNumber("gyro angle", m_gyro.getYaw());
        SmartDashboard.putBoolean("is connecgted", m_gyro.isConnected());


    }

}
