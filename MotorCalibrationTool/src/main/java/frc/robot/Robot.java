/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
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

  double axisVal;

  long firstTime = 0;
  long currentTime = 0;
  double setSpeed;
  CANSparkMax motor1;
  CANSparkMax motor2;
  CANSparkMax motor3;
  CANEncoder encoder1;
  CANEncoder encoder2;
  CANEncoder encoder3;
  Joystick stick;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor1 = new CANSparkMax(1, MotorType.kBrushless);
    motor2 = new CANSparkMax(2, MotorType.kBrushless);
    motor3 = new CANSparkMax(3, MotorType.kBrushless);
    encoder1 = new CANEncoder(motor1);
    encoder2 = new CANEncoder(motor2);
    encoder3 = new CANEncoder(motor3);
    stick = new Joystick(0);

    //motor1.setIdleMode(IdleMode.kCoast);
    //motor2.setIdleMode(IdleMode.kCoast);
    //motor2.follow(motor1, true);
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
    SmartDashboard.putNumber("Axis", axisVal);
    SmartDashboard.putNumber("Out", motor1.getAppliedOutput());
    SmartDashboard.putNumber("BusVoltage", motor1.getBusVoltage());
    SmartDashboard.putNumber("Applied Voltage", motor1.getAppliedOutput() * motor1.getBusVoltage());
    SmartDashboard.putNumber("Vel1", encoder1.getVelocity());
    SmartDashboard.putNumber("Vel2", encoder2.getVelocity());
    
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
    axisVal = ((-1.0 * stick.getRawAxis(2)) + 1.0) / 2.0; //((-1.0 * stick.getRawAxis(1)) + 1.0) / 2.0
    
  
    currentTime = System.currentTimeMillis();
    setSpeed = (double)((((currentTime - firstTime) / 1000) * 10) * 0.00021);
    // if (setSpeed < 1) {
    //   motor1.set(setSpeed);
    // } else {
    //   motor1.set(-1*axisVal);
    // }

    motor1.set(axisVal);
    motor2.set(-1 * axisVal);

    SmartDashboard.putNumber("Axis", axisVal);
    SmartDashboard.putNumber("Out", motor1.getAppliedOutput());
    SmartDashboard.putNumber("BusVoltage", motor1.getBusVoltage());
    SmartDashboard.putNumber("Applied Voltage", motor1.getAppliedOutput() * motor1.getBusVoltage());
    SmartDashboard.putNumber("Vel1", encoder1.getVelocity());
    SmartDashboard.putNumber("Vel2", encoder2.getVelocity());
    SmartDashboard.putNumber("Current", motor1.getOutputCurrent());
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
    motor3.set(stick.getRawAxis(2));
    SmartDashboard.putNumber("Vel1", encoder3.getVelocity());
  }
}
