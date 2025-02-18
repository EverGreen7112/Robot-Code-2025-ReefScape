package frc.robot.Commands.Swerve.Reef;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class AlignToBranchCommand extends Command{

    private final double POS_ERROR_TOLERANCE = 0.015;
    private final double ANGLE_ERROR_TOLERANCE = 0.5;

    private Pose2d m_target;
    private ReefFace m_reefFace;
    private boolean m_isRightBranch;
    private PIDController m_xController;
    private PIDController m_yController;

    public AlignToBranchCommand(ReefFace reefFace, boolean isRightBranch) {
        addRequirements(Swerve.getInstance());
        m_xController = new PIDController(0.2, 0, 0);
        m_yController = new PIDController(0.2, 0, 0);

        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {
        m_target = (m_isRightBranch) ? m_reefFace.getRightBranchRobotPose() : m_reefFace.getLeftBranchRobotPose();
        SwerveAngleController.getInstance().start(m_target.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        double xOutput = m_xController.calculate(pose.getX(), m_target.getX());
        double yOutput = m_yController.calculate(pose.getY(), m_target.getY());

        if(Math.abs(pose.getX() - m_target.getX()) < POS_ERROR_TOLERANCE)   
           xOutput = 0;
        if(Math.abs(pose.getY() - m_target.getY()) < POS_ERROR_TOLERANCE)
           yOutput = 0;


        Vector2d fieldOrientedVel = new Vector2d(xOutput, yOutput);
        fieldOrientedVel.rotate(pose.getRotation().getRadians());
        Swerve.getInstance().driveByVelocity(fieldOrientedVel, false);

    }

    @Override
    public boolean isFinished() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        return (Math.abs(pose.getX() - m_target.getX()) < POS_ERROR_TOLERANCE && 
                Math.abs(pose.getY() - m_target.getY()) < POS_ERROR_TOLERANCE &&
                Math.abs(pose.getRotation().getDegrees() - m_target.getRotation().getDegrees()) < ANGLE_ERROR_TOLERANCE);

    }
    
    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
        SmartDashboard.putBoolean("dsa", true);
    }



}
