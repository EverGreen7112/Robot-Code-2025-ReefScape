package frc.robot.Subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.Dispenser.AutoReadyForCoralCommand;
import frc.robot.Commands.Dispenser.WaitUntilCoralIsInCommand;
import frc.robot.Commands.Dispenser.WaitUntilCoralIsOutCommand;
import frc.robot.Commands.Elevator.MoveElevatorTo;
import frc.robot.Commands.Elevator.WaitUntilElevatorAt;
import frc.robot.Commands.Swerve.AutoDrive.AlignToBranchCommand;
import frc.robot.Commands.Swerve.AutoDrive.AlignToBranchInAutoCommand;
import frc.robot.Subsystems.Dispenser.Dispenser;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Utils.ReefFace;

public class SwerveAutoController {

    public static boolean isRobotAligning = false;

    private static final PIDConstants TRANSLATION_PID =  new PIDConstants(5.0, 0.0, 0.0),
                                      ROTATION_PID = new PIDConstants(1.0, 0.0 ,0.0);
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3, 3, 1 * Math.PI, 4 * Math.PI);
    private static final double GOAL_END_VELOCITY = 0;

    private static SwerveAutoController m_instance = new SwerveAutoController();
    private SendableChooser<Command> m_autoChooser;
    private SendableChooser<Alliance> m_allianceChooser;

    private SwerveAutoController(){

        RobotConfig config = null;

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            SmartDashboard.putBoolean("couldnt load robot config, expect problems in auto", false);
        }

        AutoBuilder.configure(
            SwerveLocalizer.getInstance()::getCurrentPoint, 
            SwerveLocalizer.getInstance()::setCurrentPoint, 
            Swerve.getInstance()::getRobotOrientedSpeeds, 
            ((speeds, feedforwards) -> Swerve.getInstance().driveRobotOrientedBySpeeds(speeds)), 
            new PPHolonomicDriveController( 
                TRANSLATION_PID, 
                ROTATION_PID 
            ),
            config, 
            () -> { //flip path
              return  getAlliance() == DriverStation.Alliance.Red;
            },
            Swerve.getInstance()
        );

        configureCommands(); //configure commands must be registered before the creation of any paths
        

        PathPlannerAuto left = new PathPlannerAuto("left 3 L4");

        left.event("aa").onTrue(new MoveElevatorTo(ElevatorLevel.L4));

        m_autoChooser = new SendableChooser<Command>();
        m_autoChooser.addOption("middle", new PathPlannerAuto("Middle 1 L4"));
        m_autoChooser.addOption("right", new PathPlannerAuto("right 3 L4"));
        m_autoChooser.addOption("left", left);
        m_autoChooser.addOption("test", new PathPlannerAuto("test"));
        

        m_allianceChooser = new SendableChooser<Alliance>();
        m_allianceChooser.addOption("blue", Alliance.Blue);
        m_allianceChooser.addOption("red", Alliance.Red);

    }

    public static SwerveAutoController getInstance(){
        return m_instance;
    }

    public void addChoosersToDashboard(){
        SmartDashboard.putData("auto", m_autoChooser);
        SmartDashboard.putData("alliance", m_allianceChooser);
    }

    public Command getAutoCommand(){
        return m_autoChooser.getSelected();
    }

    public Alliance getAlliance(){
        return m_allianceChooser.getSelected();
    }
    

    public Command generateDriveToCommand(Pose2d waypoint){
        return AutoBuilder.pathfindToPose(waypoint, PATH_CONSTRAINTS, GOAL_END_VELOCITY);
    };
    
    public Command generateDriveToCommand(Pose2d waypoint, double goalEndVelocity){
        return AutoBuilder.pathfindToPose(waypoint, PATH_CONSTRAINTS, goalEndVelocity);
    };

    public void configureCommands(){
        NamedCommands.registerCommand("MoveElevatorToL1", new MoveElevatorTo(ElevatorLevel.L1));
        NamedCommands.registerCommand("MoveElevatorToL2", new MoveElevatorTo(ElevatorLevel.L2));
        NamedCommands.registerCommand("MoveElevatorToL3", new MoveElevatorTo(ElevatorLevel.L3));
        NamedCommands.registerCommand("MoveElevatorToL4", new MoveElevatorTo(ElevatorLevel.L4));
        NamedCommands.registerCommand("MoveElevatorToGround", new MoveElevatorTo(ElevatorLevel.CLOSED));
        
        NamedCommands.registerCommand("WaitUntilElevatorL1", new WaitUntilElevatorAt(ElevatorLevel.L1));
        NamedCommands.registerCommand("WaitUntilElevatorL2", new WaitUntilElevatorAt(ElevatorLevel.L2));
        NamedCommands.registerCommand("WaitUntilElevatorL3", new WaitUntilElevatorAt(ElevatorLevel.L3));
        NamedCommands.registerCommand("WaitUntilElevatorL4", new WaitUntilElevatorAt(ElevatorLevel.L4));
        
        NamedCommands.registerCommand("WaitUntilCoralIsIn", new WaitUntilCoralIsInCommand());
        NamedCommands.registerCommand("WaitUntilCoralIsOut", new WaitUntilCoralIsOutCommand());

        NamedCommands.registerCommand("DispenceCoral", new InstantCommand(() -> {Dispenser.getInstance().dispenseCoral();}));
        NamedCommands.registerCommand("SlowDispense", new InstantCommand(() -> {Dispenser.getInstance().slowDispense();}));
        NamedCommands.registerCommand("StopDispense", new InstantCommand(() -> {Dispenser.getInstance().stop();}));
        NamedCommands.registerCommand("TurnOnLeds", new AutoReadyForCoralCommand());
        
        
    }
}
