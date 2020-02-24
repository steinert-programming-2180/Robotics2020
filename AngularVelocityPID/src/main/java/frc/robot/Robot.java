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
  TalonSRX[] leftMotors = new TalonSRX[3];
  TalonSRX[] rightMotors = new TalonSRX[3];
  AHRS ahrs;
  Double Kp = 0.0;
  Double Ki = 0.0;
  Double Kd = 0.0;
  PIDController controller;
  Joystick driver = new Joystick(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    left1 = new TalonSRX(1);
    left2 = new TalonSRX(2);
    left3 = new TalonSRX(3);
    right1 = new TalonSRX(4);
    right2 = new TalonSRX(5);
    right3 = new TalonSRX(6);

    leftMotors[0] = left1;
    leftMotors[1] = left2;
    leftMotors[2] = left3;

    rightMotors[0] = right1;
    rightMotors[1] = right2;
    rightMotors[2] = right3;

    ahrs = new AHRS(SPI.Port.kMXP);
    controller = new PIDController(Kp, Ki, Kd);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    controller.setSetpoint(ahrs.getRawGyroZ()+5);
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("RotationalVelocity", ahrs.getRawGyroZ());
    for(TalonSRX i : leftMotors){
      i.set(ControlMode.PercentOutput, driver.getRawAxis(2));
      //MathUtil.clamp(controller.calculate(ahrs.getRawGyroZ()), -1, 1)
      SmartDashboard.putNumber("BUS", i.getBusVoltage());
    }

    for(TalonSRX i : rightMotors){
      i.set(ControlMode.PercentOutput, driver.getRawAxis(2));
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
