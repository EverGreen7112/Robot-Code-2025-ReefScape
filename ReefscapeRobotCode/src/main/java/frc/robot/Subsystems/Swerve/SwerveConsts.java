package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;
import frc.robot.Utils.Math.Vector2d;

/*
 * TL = Top Left
 * TR = Top Right
 * DL = Down Left
 * DR = Down Right
 */
public interface SwerveConsts{

    public static final boolean DEBUG_MODE = false;    
    // speed values
    public static final double MAX_NORMAL_DRIVE_SPEED = 2.6; // m/s
    public static final double MAX_TURBO_DRIVE_SPEED = 4;
    public static final double MAX_SLOW_DRIVE_SPEED = 0.45;
    public static final double MAX_ANGULAR_SPEED = 180; // deg/s/
    public static final double MIN_SPEED = 0.0;
    
    public static final double GYRO_DIRECTION = -1; //decide the direction of the gyro(counter clock wise should be positive)
    
    public static final SwerveModule[] MODULES = new SwerveModule[4];

    // motor controllers
    public static final EverTalonFX 
            TL_DRIVE_MOTOR = new EverTalonFX(18),
            TR_DRIVE_MOTOR = new EverTalonFX(1), 
            DL_DRIVE_MOTOR = new EverTalonFX(16),  
            DR_DRIVE_MOTOR = new EverTalonFX(3); 
    
    public static final EverSparkMax 
            TL_STEER_MOTOR = new EverSparkMax(11),
            TR_STEER_MOTOR = new EverSparkMax(17),
            DL_STEER_MOTOR = new EverSparkMax(10),
            DR_STEER_MOTOR = new EverSparkMax(2);

    public static final EverTalonFX[] DRIVE_MOTORS = {TL_DRIVE_MOTOR, TR_DRIVE_MOTOR, DL_DRIVE_MOTOR, DR_DRIVE_MOTOR};
    public static final EverSparkMax[] STEER_MOTORS = {TL_STEER_MOTOR, TR_STEER_MOTOR, DL_STEER_MOTOR, DR_STEER_MOTOR};

    // encoders
    public static final EverTalonFXInternalEncoder 
            TL_DRIVE_ENCODER = new EverTalonFXInternalEncoder(TL_DRIVE_MOTOR),
            TR_DRIVE_ENCODER = new EverTalonFXInternalEncoder(TR_DRIVE_MOTOR),
            DL_DRIVE_ENCODER = new EverTalonFXInternalEncoder(DL_DRIVE_MOTOR),
            DR_DRIVE_ENCODER = new EverTalonFXInternalEncoder(DR_DRIVE_MOTOR);

    public static final EverSparkInternalEncoder 
            TL_STEER_ENCODER = new EverSparkInternalEncoder(TL_STEER_MOTOR),
            TR_STEER_ENCODER = new EverSparkInternalEncoder(TR_STEER_MOTOR),
            DL_STEER_ENCODER = new EverSparkInternalEncoder(DL_STEER_MOTOR),
            DR_STEER_ENCODER = new EverSparkInternalEncoder(DR_STEER_MOTOR);

    public static final EverTalonFXInternalEncoder[] DRIVE_ENCODERS = {TL_DRIVE_ENCODER, TR_DRIVE_ENCODER, DL_DRIVE_ENCODER, DR_DRIVE_ENCODER};
    public static final EverSparkInternalEncoder[] STEER_ENCODERS = {TL_STEER_ENCODER, TR_STEER_ENCODER, DL_STEER_ENCODER, DR_STEER_ENCODER};

    // swerve module pid controllers
    public static final EverTalonFXPIDController 
            TL_VELOCITY_CONTROLLER = new EverTalonFXPIDController(TL_DRIVE_MOTOR),
            TR_VELOCITY_CONTROLLER = new EverTalonFXPIDController(TR_DRIVE_MOTOR),
            DL_VELOCITY_CONTROLLER = new EverTalonFXPIDController(DL_DRIVE_MOTOR),
            DR_VELOCITY_CONTROLLER = new EverTalonFXPIDController(DR_DRIVE_MOTOR);

    public static final EverSparkMaxPIDController                 
            TL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TL_STEER_MOTOR),
            TR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TR_STEER_MOTOR),
            DL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DL_STEER_MOTOR),
            DR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DR_STEER_MOTOR);
    
    public static final EverTalonFXPIDController[] WHEEL_VELOCITY_CONTROLLERS = {TL_VELOCITY_CONTROLLER, TR_VELOCITY_CONTROLLER, DL_VELOCITY_CONTROLLER, DR_VELOCITY_CONTROLLER};
    public static final EverSparkMaxPIDController[] WHEEL_ANGLE_CONTROLLERS = {TL_ANGLE_CONTROLLER, TR_ANGLE_CONTROLLER, DL_ANGLE_CONTROLLER, DR_ANGLE_CONTROLLER};
            
    // chassis encoders 
    public static final EverAbsEncoder
            TL_ABS_ENCODER = new EverCANCoder(1),
            TR_ABS_ENCODER = new EverCANCoder(0),
            DL_ABS_ENCODER = new EverCANCoder(3),
            DR_ABS_ENCODER = new EverCANCoder(2);

    public static final EverAbsEncoder[] ABS_ENCODERS = {TL_ABS_ENCODER, TR_ABS_ENCODER, DL_ABS_ENCODER, DR_ABS_ENCODER};
    

    // swerve module velocity pidf values
    public static final double WHEEL_VELOCITY_KP = 0.1, WHEEL_VELOCITY_KI = 0.0, WHEEL_VELOCITY_KD = 0.00,
            WHEEL_VELOCITY_KV = 1/8.5, WHEEL_VELOCITY_KS = 0;
    // swerve module wheel angle pid values
    public static final double WHEEL_ANGLE_KP = 0.01, WHEEL_ANGLE_KI = 0.0, WHEEL_ANGLE_KD = 0.000;

    // swerve dimensions
    public static final double CHASSIS_WIDTH = 0.75, CHASSIS_LENGTH = 0.75;
    public static final double BUMPERS_THICKNESS = 0.06;
    
    public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
            CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_LENGTH * CHASSIS_LENGTH);
    public static final double ROBOT_RADIUS = 0.5 * Math.sqrt(
            CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_LENGTH * CHASSIS_LENGTH);
    public static final double WHEEL_PERIMETER = Math.PI * 0.09;

    // module gear ratios
    public static final double DRIVE_GEAR_RATIO = 1 / 6.75, STEER_GEAR_RATIO = 1 / 12.8;

    // swerve vectors
    public static final Vector2d 
            TR = new Vector2d((CHASSIS_WIDTH / 2),
                    (CHASSIS_LENGTH / 2)),
            TL = new Vector2d(-(CHASSIS_WIDTH / 2),
                    (CHASSIS_LENGTH / 2)),
            DR = new Vector2d(CHASSIS_WIDTH / 2,
                    -(CHASSIS_LENGTH / 2)),
            DL = new Vector2d(-(CHASSIS_WIDTH / 2),
                    -(CHASSIS_LENGTH / 2));

    // array of physical module vectors
    public static final Vector2d[] modulesPositions = { TL,
                                                                 TR,
                                                                 DL,
                                                                 DR
    };// array of vectors from robot center to swerves module
  

    
    public static void config(){
        for (EverMotorController driveMotor : DRIVE_MOTORS) {
                driveMotor.restoreFactoryDefaults();
                driveMotor.setInverted(false);
                driveMotor.setIdleMode(IdleMode.kCoast);
           }
           
           for (EverMotorController steerMotor : STEER_MOTORS) {
               steerMotor.restoreFactoryDefaults();
               steerMotor.setIdleMode(IdleMode.kCoast);
           }
           
           for(EverEncoder driveEncoder : DRIVE_ENCODERS){
               driveEncoder.setVelConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER);
               driveEncoder.setPosConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER);
           }
   
           for(EverEncoder steerEncoder : STEER_ENCODERS){
               steerEncoder.setPosConversionFactor(SwerveConsts.STEER_GEAR_RATIO * 360.0); //rotations to degrees
           }
   
           for(EverAbsEncoder absEncoder : ABS_ENCODERS){
               absEncoder.setPosConversionFactor(360.0);
           }
   
           ABS_ENCODERS[0].setOffset(80.507); //80.5078125
           ABS_ENCODERS[1].setOffset(137.37); //-42.275394439697266
           ABS_ENCODERS[2].setOffset(-149.150); //-149.50196838378906
           ABS_ENCODERS[3].setOffset(-27.509); //-34.8046875
       
   
           for (EverTalonFXPIDController velocityController : WHEEL_VELOCITY_CONTROLLERS) {
               Slot0Configs configs = new Slot0Configs();
               configs.kP = WHEEL_VELOCITY_KP;
               configs.kI = WHEEL_VELOCITY_KI;
               configs.kD = WHEEL_VELOCITY_KD;
               configs.kS = WHEEL_VELOCITY_KS;
               configs.kV = WHEEL_VELOCITY_KV;
               velocityController.setPID(configs);   
           }
   
           for (EverSparkMaxPIDController angleController : WHEEL_ANGLE_CONTROLLERS) {
                angleController.setPID(WHEEL_ANGLE_KP, WHEEL_ANGLE_KI, WHEEL_ANGLE_KD);      
           }

        MODULES[0] = new SwerveModule(SwerveConsts.TL_VELOCITY_CONTROLLER, SwerveConsts.TL_DRIVE_MOTOR, SwerveConsts.TL_DRIVE_ENCODER, SwerveConsts.TL_ANGLE_CONTROLLER, SwerveConsts.TL_STEER_MOTOR, SwerveConsts.TL_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[0]);
        MODULES[1] = new SwerveModule(SwerveConsts.TR_VELOCITY_CONTROLLER, SwerveConsts.TR_DRIVE_MOTOR, SwerveConsts.TR_DRIVE_ENCODER, SwerveConsts.TR_ANGLE_CONTROLLER, SwerveConsts.TR_STEER_MOTOR, SwerveConsts.TR_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[1]);
        MODULES[2] = new SwerveModule(SwerveConsts.DL_VELOCITY_CONTROLLER, SwerveConsts.DL_DRIVE_MOTOR, SwerveConsts.DL_DRIVE_ENCODER, SwerveConsts.DL_ANGLE_CONTROLLER, SwerveConsts.DL_STEER_MOTOR, SwerveConsts.DL_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[2]);
        MODULES[3] = new SwerveModule(SwerveConsts.DR_VELOCITY_CONTROLLER, SwerveConsts.DR_DRIVE_MOTOR, SwerveConsts.DR_DRIVE_ENCODER, SwerveConsts.DR_ANGLE_CONTROLLER, SwerveConsts.DR_STEER_MOTOR, SwerveConsts.DR_STEER_ENCODER, SwerveConsts.ABS_ENCODERS[3]);
    }

}
