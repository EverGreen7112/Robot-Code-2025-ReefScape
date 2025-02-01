package frc.robot.Commands.Swerve.Reef;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Vector2d;

public class AlignToBranchCommand extends Command{

    private final double ERROR_TOLERANCE = 0.5;

    private Pose2d m_target;
    private PIDController m_xController;
    private PIDController m_yController;

    public AlignToBranchCommand(ReefFace reefFace, boolean isRightBranch) {
        addRequirements(Swerve.getInstance());
        m_target = (isRightBranch) ? reefFace.getRightBranchPose():  reefFace.getLeftBranchPose();
        m_xController = new PIDController(1.7, 0, 0);
        m_yController = new PIDController(1.7, 0, 0);
        
    }

    @Override
    public void execute() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        double xOutput = -m_xController.calculate(pose.getX(), m_target.getX());
        double yOutput = -m_yController.calculate(pose.getY(), m_target.getY());

        if(Math.abs(pose.getX() - m_target.getX()) < ERROR_TOLERANCE)
           xOutput = 0;
        if(Math.abs(pose.getY() - m_target.getY()) < ERROR_TOLERANCE)
           yOutput = 0;

        Swerve.getInstance().driveByVelocity(new Vector2d(xOutput, yOutput), true);
        SwerveAngleController.getInstance().start(m_target.getRotation().getDegrees(), true);
    }

    @Override
    public boolean isFinished() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        return Math.abs(pose.getX() - m_target.getX()) < ERROR_TOLERANCE && Math.abs(pose.getY() - m_target.getY()) < ERROR_TOLERANCE; 
    }
    
    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
    }



}
