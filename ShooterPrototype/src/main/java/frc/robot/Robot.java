/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
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
  CANSparkMax motor1 = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(6, MotorType.kBrushless);
  //CANSparkMax motor2 = new CANSparkMax(6, MotorType.kBrushless);
  CANPIDController pidController = new CANPIDController(motor1);
  CANEncoder encoder = new CANEncoder(motor1);

  double modAxis;

  Joystick stick = new Joystick(0);

  double kP = 0.0005;
  double kI = 0.0;
  double kD = 0.0;
  double kF = 0.00215;
  double kIz = 0.0;
  double kMaxOutput = 1.0;
  double kMinOutput = -1.0;

  double targetSpeed;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor1.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);
    motor1.setInverted(true);
    motor2.follow(motor1, true);
    //motor2.setIdleMode(IdleMode.kCoast);
    //motor2.follow(motor1, true); //Sets to follow, and has 'inverted' set to true

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
  public void teleopInit() {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void teleopPeriodic() {
    targetSpeed = ((stick.getRawAxis(2) + 1.0) / 2.0) * 5000.0;
    pidController.setFF(0.00215 / motor1.getBusVoltage());
    pidController.setReference(targetSpeed, ControlType.kVelocity);
    //motor1.set(targetSpeed / 5000.0);
    SmartDashboard.putNumber("Target", targetSpeed);
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Error", encoder.getVelocity() - targetSpeed);
    SmartDashboard.putNumber("Applied Percent", motor1.getAppliedOutput());
    SmartDashboard.putNumber("Ratio", (motor1.getAppliedOutput() * motor1.getBusVoltage()) / (encoder.getVelocity() + 0.000000001));
    SmartDashboard.putNumber("Bus Voltage", motor1.getBusVoltage());
  }

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

   

    
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
  public void autonomousPeriodic() {
    
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
