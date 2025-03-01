package frc.robot.Commands.Swerve.AutoDrive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;

public class DriveToCoralStationCommand extends Command {

    private boolean m_isRight, m_isInner;
    private Command m_driveCommand;
    private Pose2d[][] m_coralStations;

    public DriveToCoralStationCommand(boolean isRight, boolean isInner) {
        this.m_isRight = isRight;
        this.m_isInner = isInner;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = SwerveLocalizer.getInstance().getCurrentPoint();
        
        //use pathplanner only for long distances
        int row = (m_isInner) ? (0) : (1);
        int colomm = (m_isRight) ? (0) : (1);
        
        Pose2d coralStation = m_coralStations[row][colomm];

        Command d =  SwerveAutoController.getInstance().generateDriveToCommand(coralStation);
        m_driveCommand = SwerveAutoController.getInstance().generateDriveToCommand(coralStation);

        m_driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveCommand.cancel();
    }
}
