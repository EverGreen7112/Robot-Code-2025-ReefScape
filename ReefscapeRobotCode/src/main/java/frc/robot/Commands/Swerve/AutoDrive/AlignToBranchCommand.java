package frc.robot.Commands.Swerve.AutoDrive;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Commands.Elevator.MoveElevatorToSelectedLevel;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class AlignToBranchCommand extends Command{

    private final double POS_ERROR_TOLERANCE = 0.01;
    private final double ANGLE_ERROR_TOLERANCE = 0.5;

    private Pose2d m_target;
    private ReefFace m_reefFace;
    private boolean m_isRightBranch;
    private ProfiledPIDController m_xController;
    private ProfiledPIDController m_yController;

    public AlignToBranchCommand(ReefFace reefFace, boolean isRightBranch) {
        addRequirements(Swerve.getInstance());
        m_xController = new ProfiledPIDController(3, 0, 0, new Constraints(1, 1));
        m_yController = new ProfiledPIDController(3, 0, 0, new Constraints(1, 1));
    
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {
        m_target = (m_isRightBranch) ? m_reefFace.getRightBranchRobotPose() : m_reefFace.getLeftBranchRobotPose();
        SwerveAngleController.getInstance().start(m_target.getRotation().getDegrees(), true);

        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        m_xController.reset(pose.getX());
        m_yController.reset(pose.getY());

        (new MoveElevatorToSelectedLevel()).schedule();;
    }

    @Override
    public void execute() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        double xOutput = m_xController.calculate(pose.getX(), m_target.getX());
        double yOutput = m_yController.calculate(pose.getY(), m_target.getY());

        SmartDashboard.putNumber("x", pose.getX());
        SmartDashboard.putNumber("y", pose.getY());

        if(Math.abs(pose.getX() - m_target.getX()) < POS_ERROR_TOLERANCE)   
           xOutput = 0;
        if(Math.abs(pose.getY() - m_target.getY()) < POS_ERROR_TOLERANCE)
           yOutput = 0;


        Vector2d fieldOrientedVel = new Vector2d(xOutput, yOutput);
        fieldOrientedVel.rotate(pose.getRotation().getRadians() * SwerveConsts.GYRO_DIRECTION);
        Swerve.getInstance().driveByVelocity(fieldOrientedVel, false);

    }

    @Override
    public boolean isFinished() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        return  MathUtil.isNear(m_target.getX(), pose.getX(), POS_ERROR_TOLERANCE) && 
                MathUtil.isNear(m_target.getY(), pose.getY(), POS_ERROR_TOLERANCE) &&
                MathUtil.isNear(m_target.getRotation().getDegrees(), pose.getRotation().getDegrees(), ANGLE_ERROR_TOLERANCE);

    }
    
    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
        Swerve.getInstance().stop();


    }



}
