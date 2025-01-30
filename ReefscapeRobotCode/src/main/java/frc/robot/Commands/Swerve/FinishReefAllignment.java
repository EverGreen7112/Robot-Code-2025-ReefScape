package frc.robot.Commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Vector2d;

public class FinishReefAllignment extends Command{
    private ReefFace m_reefFace;
    private boolean m_isRightBranch;
    private Pose2d m_target;
    private PIDController m_xController;
    private PIDController m_yController;
    private PIDController m_angleController;



    public FinishReefAllignment(ReefFace reefFace, boolean isRightBranch) {
        addRequirements(Swerve.getInstance());
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
        m_target = (isRightBranch) ? reefFace.getRightBranchPose():  reefFace.getLeftBranchPose();
        m_xController = new PIDController(4, 0, 0);
        m_yController = new PIDController(4, 0, 0);
        
    }

    @Override
    public void execute() {
        Pose2d pose = SwerveLocalizer.getInstance().getCurrentPoint();
        double xOutput = m_xController.calculate(pose.getX(), m_target.getX());
        double yOutput = m_yController.calculate(pose.getY(), m_target.getY());
        
        Swerve.getInstance().driveByVelocity(new Vector2d(xOutput, yOutput), true);
        SwerveAngleController.getInstance().start(m_target.getRotation().getDegrees(), true);
    }
    



}
