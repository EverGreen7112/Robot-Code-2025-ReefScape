package frc.robot.Commands.Swerve.Reef;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.ReefFace;

public class DriveToBranchCommand extends Command {

    private ReefFace m_reefFace;
    private boolean m_isRightBranch;
    private Command m_driveCommand;
    private BooleanSupplier m_stopCommand;

    public DriveToBranchCommand(ReefFace reefFace, boolean isRightBranch, BooleanSupplier stopCommand) {
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
        m_stopCommand = stopCommand;
    }

    @Override
    public void initialize() {
        Pose2d targetBranch = (m_isRightBranch) ? m_reefFace.getRightBranchRobotPose() : m_reefFace.getLeftBranchRobotPose();

        m_driveCommand = SwerveAutoController.getInstance().generateDriveToCommand(
                new GoalEndState(0, m_reefFace.getFacePose().getRotation()),
                targetBranch.plus(new Transform2d(-0.1, 0, new Rotation2d())),
                targetBranch).andThen(new AlignToBranchCommand(m_reefFace, m_isRightBranch));

        m_driveCommand.schedule();

    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled() || m_stopCommand.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveCommand.cancel();
    }
}
