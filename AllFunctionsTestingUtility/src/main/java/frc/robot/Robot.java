/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  Joystick stick1 = new Joystick(0);
  Joystick stick2 = new Joystick(1);

  CANSparkMax left1, left2, left3, right1, right2, right3, elevator1, elevator2, shoot1, shoot2, paddy, funnel, conveyer;
  TalonSRX leftIntake, rightIntake;

  DigitalInput beamTrip1, beamTrip2, beamTrip3, beamTrip4, beamTrip5, beamTrip6;

  DoubleSolenoid intakePiston1, intakePiston2;
  Compressor compressor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    left1 = new CANSparkMax(1, MotorType.kBrushless);
    // left1.setInverted(false);
    left2 = new CANSparkMax(2, MotorType.kBrushless);
    // left2.follow(left1, false);
    left3 = new CANSparkMax(3, MotorType.kBrushless);
    // left2.follow(left1, false);
    right1 = new CANSparkMax(4, MotorType.kBrushless);
    // right1.setInverted(false);
    right2 = new CANSparkMax(5, MotorType.kBrushless);
    // right2.follow(right1, false);
    right3 = new CANSparkMax(6, MotorType.kBrushless);
    // right3.follow(right1, false);
    elevator1 = new CANSparkMax(7, MotorType.kBrushless);
    // elevator1.setInverted(false);
    elevator2 = new CANSparkMax(8, MotorType.kBrushless);
    // elevator2.follow(elevator1, true);
    shoot1 = new CANSparkMax(9, MotorType.kBrushless);
    // shoot1.setInverted(false);
    shoot2 = new CANSparkMax(10, MotorType.kBrushless);
    // shoot2.follow(shoot1, false);
    paddy = new CANSparkMax(11, MotorType.kBrushless);
    // paddy.setInverted(false);
    funnel = new CANSparkMax(12, MotorType.kBrushless);
    // funnel.setInverted(false);
    conveyer = new CANSparkMax(13, MotorType.kBrushless);
    // conveyer.setInverted(false);

    leftIntake = new TalonSRX(14);
    // leftIntake.setInverted(InvertType.InvertMotorOutput);
    rightIntake = new TalonSRX(15);
    // rightIntake.follow(leftIntake);
    // rightIntake.setInverted(InvertType.OpposeMaster);

    intakePiston1 = new DoubleSolenoid(0,1);
    intakePiston2 = new DoubleSolenoid(2,3);

    beamTrip1 = new DigitalInput(0);
    beamTrip2 = new DigitalInput(1);
    beamTrip3 = new DigitalInput(2);
    beamTrip4 = new DigitalInput(3);
    beamTrip5 = new DigitalInput(4);
    beamTrip6 = new DigitalInput(5);

    compressor = new Compressor();
    compressor.start();

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
    double throttle = stick1.getRawAxis(2) * -1;
    if (stick1.getRawButton(1)) {
      left1.set(throttle);
    } if (stick1.getRawButton(2)) {
      left2.set(throttle);
    } if (stick1.getRawButton(3)) {
      left3.set(throttle);
    } if (stick1.getRawButton(4)) {
      right1.set(throttle);
    } if (stick1.getRawButton(5)) {
      right2.set(throttle);
    } if (stick1.getRawButton(6)) {
      right3.set(throttle);
    } if (stick1.getRawButton(7)) {
      elevator1.set(throttle);
    } if (stick1.getRawButton(8)) {
      elevator2.set(throttle);
    } if (stick1.getRawButton(9)) {
      shoot1.set(throttle);
    } if (stick2.getRawButton(1)) {
      shoot2.set(throttle);
    } if (stick2.getRawButton(2)) {
      paddy.set(throttle);
    } if (stick2.getRawButton(3)) {
      conveyer.set(throttle);
    } if (stick2.getRawButton(4)) {
      funnel.set(throttle);
    } if (stick2.getRawButton(5)) {
      leftIntake.set(ControlMode.PercentOutput, throttle);
    } if (stick2.getRawButton(6)) {
      rightIntake.set(ControlMode.PercentOutput, throttle);
    } if (stick2.getRawButton(7)) {
      intakePiston1.set(Value.kForward);
    } if (stick2.getRawButton(8)) {
      intakePiston1.set(Value.kReverse);
    } if (stick2.getRawButton(9)) {
      intakePiston2.set(Value.kForward);
    } if (stick1.getRawButton(10)) {
      intakePiston2.set(Value.kForward);
    }

    SmartDashboard.putNumber("Throttle", throttle);

    SmartDashboard.putNumber("Left Speed", left1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Speed", right1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Speed", shoot1.getEncoder().getVelocity());
    SmartDashboard.putNumber("Paddy Speed", paddy.getEncoder().getVelocity());
    SmartDashboard.putNumber("Paddy Position", paddy.getEncoder().getPosition());
    SmartDashboard.putNumber("Conveyer Speed", conveyer.getEncoder().getVelocity());
    SmartDashboard.putNumber("Conveyer Position", conveyer.getEncoder().getPosition());
    SmartDashboard.putNumber("Funnel Speed", funnel.getEncoder().getVelocity());
    SmartDashboard.putNumber("Funnel Position", funnel.getEncoder().getPosition());

    SmartDashboard.putBoolean("Beam Trip 1", beamTrip1.get());
    SmartDashboard.putBoolean("Beam Trip 2", beamTrip2.get());
    SmartDashboard.putBoolean("Beam Trip 3", beamTrip3.get());
    SmartDashboard.putBoolean("Beam Trip 4", beamTrip4.get());
    SmartDashboard.putBoolean("Beam Trip 5", beamTrip5.get());
    SmartDashboard.putBoolean("Beam Trip 6", beamTrip6.get());
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
