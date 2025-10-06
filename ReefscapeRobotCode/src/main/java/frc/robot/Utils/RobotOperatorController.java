package frc.robot.Utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.EverKit.Periodic;

public class RobotOperatorController implements Periodic{
    private final static boolean DEBUG_MODE = true;
    private static RobotOperatorController m_instance = new RobotOperatorController();

    private static NetworkTableInstance m_networkTableInst;
    private static NetworkTable m_table;

    private static DoubleSubscriber m_branchSubscriber;
    private static DoubleSubscriber m_elevatorSubscriber;
    private static BooleanPublisher m_allince;
    private static BooleanSubscriber m_ledSubscriber;

    private static double m_branch = 0;
    private static double m_elevatorLevel = 0;
    private static boolean m_led = false;

    public RobotOperatorController(){
        m_networkTableInst = NetworkTableInstance.getDefault();

        m_table = m_networkTableInst.getTable("RobotController");

        m_branchSubscriber = m_table.getDoubleTopic("branch").subscribe(1);
        m_elevatorSubscriber = m_table.getDoubleTopic("elevator").subscribe(1);
        m_allince = m_table.getBooleanTopic("allince").publish();
        m_ledSubscriber = m_table.getBooleanTopic("led").subscribe(false);

        m_networkTableInst.startServer();
    }   

    public static RobotOperatorController getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        m_branch = m_branchSubscriber.get();
        m_elevatorLevel = m_elevatorSubscriber.get();
        m_led = m_ledSubscriber.get();
        boolean allince = (SwerveAutoController.getInstance().getAlliance() == Alliance.Blue) ? true : false;
        m_allince.set(allince);

        if(DEBUG_MODE)
            log();
    }

    public double getBranch(){
        return m_branch;
    }

    public double getElevatorLevel(){
        return m_elevatorLevel;
    }

    public boolean getLed(){
        return m_led;
    }

    private void log(){
        SmartDashboard.putNumber("selected branch", RobotOperatorController.getInstance().getBranch());
        SmartDashboard.putNumber("selected elevator level", RobotOperatorController.getInstance().getElevatorLevel());
    }

    public void initialize(){
        start(Periodic.PeriodicTime.kRobotPeriodic);
    }

}
