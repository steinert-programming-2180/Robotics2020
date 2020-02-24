/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Joystick leftStick, rightStick;
  CANSparkMax left1, left2, left3, right1, right2, right3;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);

    left1 = new CANSparkMax(1, MotorType.kBrushless);
    left2 = new CANSparkMax(2, MotorType.kBrushless);
    left3 = new CANSparkMax(3, MotorType.kBrushless);
    left1.setInverted(true);
    left2.follow(left1, false);
    left3.follow(left1, false);
    
    right1 = new CANSparkMax(4, MotorType.kBrushless);
    right2 = new CANSparkMax(5, MotorType.kBrushless);
    right3 = new CANSparkMax(6, MotorType.kBrushless);
    right1.setInverted(false);
    right2.follow(right1, false);
    right3.follow(right1, false);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("leftJoystick", leftStick.getRawAxis(1));
    SmartDashboard.putNumber("rightJoystick", rightStick.getRawAxis(1));

    left1.set(leftStick.getRawAxis(1) * 0.2);
    right1.set(rightStick.getRawAxis(1) * 0.2);

    // while(leftStick.getRawButton(1)){
    //   left1.set(leftStick.getRawAxis(1) * 0.2);
    //   right1.set(leftStick.getRawAxis(1) * 0.2);
    // }
    
    // while(leftStick.getRawButton(2)){
    //   left2.set(leftStick.getRawAxis(1) * 0.2);
    //   right2.set(leftStick.getRawAxis(1) * 0.2);
    // }

    // while(leftStick.getRawButton(3)){
    //   left3.set(leftStick.getRawAxis(1) * 0.2);
    //   right3.set(leftStick.getRawAxis(1) * 0.2);
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
