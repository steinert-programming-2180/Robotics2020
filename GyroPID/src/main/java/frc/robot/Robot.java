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
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
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
  CANSparkMax left1, left2, left3;
  CANSparkMax right1, right2, right3;
  CANSparkMax[] leftMotors = new CANSparkMax[3];
  CANSparkMax[] rightMotors = new CANSparkMax[3];

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
    left1 = new CANSparkMax(1, MotorType.kBrushless);
    left2 = new CANSparkMax(2, MotorType.kBrushless);
    left3 = new CANSparkMax(3, MotorType.kBrushless);
    right1 = new CANSparkMax(4, MotorType.kBrushless);
    right2 = new CANSparkMax(5, MotorType.kBrushless);
    right3 = new CANSparkMax(6, MotorType.kBrushless);

    leftMotors[0] = left1;
    leftMotors[1] = left2;
    leftMotors[2] = left3;

    rightMotors[0] = right1;
    rightMotors[1] = right2;
    rightMotors[2] = right3;

    controller.setP(0.003);
    controller.setI(0.0002);
    controller.setD(0.01);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);
    SmartDashboard.putNumber("D", 0);
    controller.setSetpoint(ahrs.getAngle() + 90);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("ANGLE", ahrs.getAngle());
    SmartDashboard.putNumber("PID", MathUtil.clamp(controller.calculate(ahrs.getAngle()), -1, 1));
    SmartDashboard.putNumber("Error", controller.getPositionError());

    for(CANSparkMax i : leftMotors){
      i.set(MathUtil.clamp(controller.calculate(ahrs.getAngle()), -1, 1));
    }

    for(CANSparkMax i : rightMotors){
      i.set(MathUtil.clamp(controller.calculate(ahrs.getAngle()), -1, 1));
    }
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
    SmartDashboard.putNumber("WeirdAngle", ahrs.getYaw());
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
