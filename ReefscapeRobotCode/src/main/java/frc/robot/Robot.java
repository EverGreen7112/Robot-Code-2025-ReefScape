
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Subsystems.Dispenser.Dispenser;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.Elastic;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.TalonFxCalib;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverMotionMagicPIDController;

public class Robot extends TimedRobot {

  public static ArrayList<Periodic> robotPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> teleopPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> testPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> autonomousPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> simulationPeriodicFuncs = new ArrayList<Periodic>();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private static Field2d m_field; 

  @Override
  public void robotInit() {
    
    m_robotContainer = new RobotContainer();
    // Swerve.getInstance().resetGyro();

    //create and add robot field data to dashboard
    m_field = new Field2d();
    SmartDashboard.putData("field", m_field);
    

    // SwerveAutoController.getInstance().addChoosersToDashboard();
    SmartDashboard.putNumber("target", 30);
    SmartDashboard.putNumber("kd",0.425);
    SmartDashboard.putNumber("kg", 0.5);
    SmartDashboard.putNumber("ki", 0);
    SmartDashboard.putNumber("kp", 0.25);
    SmartDashboard.putNumber("kv", 1/4.7);
    SmartDashboard.putNumber("ks", 0.1);
      
    motor.getControllerInstance().setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    for (Periodic method : robotPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

   
    // update the robot position of dashboard
    m_field.setRobotPose(SwerveLocalizer.getInstance().getCurrentPoint().getX(),
                         SwerveLocalizer.getInstance().getCurrentPoint().getY(),
                        new Rotation2d(Math.toRadians(SwerveLocalizer.getInstance().getFieldOrientedAngle())));

    // SmartDashboard.putString("pose", SwerveLocalizer.getInstance().getCurrentPoint().toString());
    SmartDashboard.putNumber("current rot", motor.getControllerInstance().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("velocity", motor.getControllerInstance().getVelocity().getValueAsDouble());

  }

  @Override
  public void disabledInit() {
    // Elevator.getInstance().m_motor.setIdleMode(frc.robot.Utils.EverKit.EverMotorController.IdleMode.kBrake);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = SwerveAutoController.getInstance().getAutoCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    for (Periodic method : autonomousPeriodicFuncs) {
      try {
        method.periodic();

      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    
  }

  @Override
  public void autonomousExit() {}

  EverTalonFX motor = new EverTalonFX(13);
  EverMotionMagicPIDController talonPidController = new EverMotionMagicPIDController(motor, 50, 55);
  Slot0Configs a = new Slot0Configs();
  

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    a.kD = SmartDashboard.getNumber("kd",0);
    a.kG = SmartDashboard.getNumber("kg", 0.45);
    a.kI = SmartDashboard.getNumber("ki", 0);
    a.kP = SmartDashboard.getNumber("kp", 0.4);
    a.kV = SmartDashboard.getNumber("kv", 1/2.6);
    a.kS = SmartDashboard.getNumber("ks", 0.2);
    double target = SmartDashboard.getNumber("target", 20);
    a.GravityType = GravityTypeValue.Elevator_Static;
    
    talonPidController.setPID(a);
    talonPidController.activate(target, ControlType.kPos);
    // motor.getControllerInstance().setControl(new VoltageOut(1));
  }

  @Override
  public void teleopPeriodic() { 

    for (Periodic method : teleopPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }   
    //Dispenser.getInstance().getCoralInPosition();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    for (Periodic method : testPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void testExit() {}  

  @Override
  public void simulationPeriodic() {
    for (Periodic method : simulationPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }


  
  
}
