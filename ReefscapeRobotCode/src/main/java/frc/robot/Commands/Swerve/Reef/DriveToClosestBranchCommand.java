package frc.robot.Commands.Swerve.Reef;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;

public class DriveToClosestBranchCommand extends Command {

    private boolean m_isRightBranch;
    private BooleanSupplier m_stopCommand;
    private Command m_driveCommand;

    public DriveToClosestBranchCommand(boolean isRightBranch, BooleanSupplier stopCommand) {
        m_isRightBranch = isRightBranch;
        m_stopCommand = stopCommand;
    }

    @Override
    public void initialize() {
        ReefFace[] reef = (SwerveAutoController.getInstance().getAlliance() == Alliance.Blue ? ReefFace.BLUE_REEF : ReefFace.RED_REEF);
        Pose2d currentPoint = SwerveLocalizer.getInstance().getCurrentPoint();
        double minDis = getDis(currentPoint, reef[0].getFacePose());

        ReefFace closestFace = reef[0];
        for (int i = 1; i < 6; i++) {
            double currentDistance = getDis(currentPoint, reef[i].getFacePose());
            if (minDis > currentDistance) {
                minDis = currentDistance;
                closestFace = reef[i];
            }
        }
        m_driveCommand = (new DriveToBranchCommand(closestFace, m_isRightBranch, m_stopCommand));
        m_driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled() || m_stopCommand.getAsBoolean();
    }


    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("false", false);
    }
    private double getDis(Pose2d first, Pose2d second){
        return (first.minus(second)).getTranslation().getNorm();
    }

}
