package frc.robot.Utils;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Utils.EverKit.Periodic;

public class RobotOperatorController implements Periodic{
    private final static boolean DEBUG_MODE = true;
    private static RobotOperatorController m_instance = new RobotOperatorController();

    private static NetworkTableInstance m_networkTableInst;
    private static NetworkTable m_table;

    private static DoubleSubscriber m_branchSubscriber;
    private static DoubleSubscriber m_elevatorSubscriber;
    private static BooleanSubscriber m_feederSubscriber;
    private static BooleanSubscriber m_innerSubscriber;

    private static double m_branch = 0;
    private static double m_elevatorLevel = 0;
    private static boolean m_feeder = true , m_inner = true;

    public RobotOperatorController(){
        m_networkTableInst = NetworkTableInstance.getDefault();

        m_table = m_networkTableInst.getTable("RobotController");

        m_branchSubscriber = m_table.getDoubleTopic("branch").subscribe(1);
        m_elevatorSubscriber = m_table.getDoubleTopic("elevator").subscribe(1);
        m_feederSubscriber = m_table.getBooleanTopic("feeder").subscribe(true);
        m_innerSubscriber = m_table.getBooleanTopic("inner").subscribe(true);

        m_networkTableInst.startServer();
        start(Periodic.PeriodicTime.kRobotPeriodic);
    }   

    public static RobotOperatorController getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        m_branch = m_branchSubscriber.get();
        m_elevatorLevel = m_elevatorSubscriber.get();
        m_feeder = m_feederSubscriber.get();
        m_inner = m_innerSubscriber.get();

        if(DEBUG_MODE)
            log();
    }

    public double getBranch(){
        return m_branch;
    }

    public double getElevatorLevel(){
        return m_elevatorLevel;
    }

    public boolean getFeeder(){
        return m_feeder;
    }

    public boolean getInner(){
        return m_inner;
    }

    private void log(){
        SmartDashboard.putNumber("selected branch", RobotOperatorController.getInstance().getBranch());
        SmartDashboard.putNumber("selected elevator level", RobotOperatorController.getInstance().getElevatorLevel());
        SmartDashboard.putBoolean("selected feeder", RobotOperatorController.getInstance().getFeeder());
        //SmartDashboard.putBoolean("is inner", RobotOperatorController.getInstance().getInner());
    }

}
