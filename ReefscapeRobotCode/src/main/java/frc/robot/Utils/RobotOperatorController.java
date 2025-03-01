package frc.robot.Utils;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Utils.EverKit.Periodic;

public class RobotOperatorController implements Periodic{
    private static RobotOperatorController m_instance = new RobotOperatorController();

    private static NetworkTableInstance m_networkTableInst;
    private static NetworkTable m_table;

    private static DoubleSubscriber m_branchSubscriber;
    private static DoubleSubscriber m_elevatorSubscriber;
    private static BooleanSubscriber m_isInnerSub, m_isRightSub;

    private static double m_branch = 0;
    private static double m_elevatorLevel = 0;
    private static boolean m_isInner = false, m_isRight = false;

    public RobotOperatorController(){
        m_networkTableInst = NetworkTableInstance.getDefault();

        m_table = m_networkTableInst.getTable("RobotController");

        m_branchSubscriber = m_table.getDoubleTopic("branch").subscribe(1d);
        m_elevatorSubscriber = m_table.getDoubleTopic("elevator").subscribe(1);
        m_isInnerSub = m_table.getBooleanTopic("feeder").subscribe(false);
        m_isRightSub = m_table.getBooleanTopic("inner").subscribe(false);


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
        m_isInner = m_isInnerSub.get();
        m_isRight = m_isRightSub.get();
    }

    public double getBranch(){
        return m_branch;
    }

    public double getElevatorLevel(){
        return m_elevatorLevel;
    }

    public boolean getIsInner(){
        return m_isInner;
    }

    public boolean getIsRight(){
        return m_isRight;
    }

}
