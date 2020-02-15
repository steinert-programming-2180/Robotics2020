/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  TalonSRX left1, left2, left3;
  TalonSRX right1, right2, right3;
  TalonSRX[] leftMotors;
  TalonSRX[] rightMotors;
  AHRS ahrs;
  ControlMode cm = ControlMode.PercentOutput;
  double Kp = -0.335938;
  double Ki = 0.2;
  double Kd = 0.3;
  double integral, previous_error, setpoint = 0;
  PIDController controller = new PIDController(Kp, Ki, Kd);
  Joystick joy = new Joystick(0);
  Joystick joy2 = new Joystick(1);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    ahrs = new AHRS(SPI.Port.kMXP);
    // left1 = new TalonSRX(1);
    // left2 = new TalonSRX(2);
    // left3 = new TalonSRX(3);
    // right1 = new TalonSRX(4);
    // right2 = new TalonSRX(5);
    // right3 = new TalonSRX(6);

    // leftMotors[0] = left1;
    // leftMotors[1] = left2;
    // leftMotors[2] = left3;

    // rightMotors[0] = right1;
    // rightMotors[1] = right2;
    // rightMotors[2] = right3;
    //controller.setTolerance(10);
    //left1.set(cm, controller.calculate(1.0));
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

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    controller.setP(SmartDashboard.getNumber("P", 0));
    controller.setI(SmartDashboard.getNumber("I", 0));
    controller.setD(SmartDashboard.getNumber("D", 0));
    SmartDashboard.putNumber("PJoystick", joy.getRawAxis(2));
    SmartDashboard.putNumber("DJoystick", joy2.getRawAxis(2));
    SmartDashboard.putNumber("ANGLE", ahrs.getAngle());
    SmartDashboard.putNumber("PID", MathUtil.clamp(controller.calculate(ahrs.getAngle()), -1, 1));
    SmartDashboard.putNumber("Error", controller.getPositionError());
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

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
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
