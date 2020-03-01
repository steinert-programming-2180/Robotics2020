/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
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
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  public double rpm;

  Joystick stick = new Joystick(0);

  private CANSparkMax motor;
  private CANSparkMax motor2;
  private CANPIDController pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, target;
  int counter = 0;

  /**
   * A CANEncoder object is constructed using the GetEncoder() method on an 
   * existing CANSparkMax object. The assumed encoder type is the hall effect,
   * or a sensor type and counts per revolution can be passed in to specify
   * a different kind of sensor. Here, it's a quadrature encoder with 4096 CPR.
   */
  private CANEncoder encoder;

  double error;
  long firstTime, secondTime;
  boolean outRange;
  double[] errorVals = new double[10];

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // initialize SPARK MAX with CAN ID
    target = 0;

    motor = new CANSparkMax(8, MotorType.kBrushless);
    motor2 = new CANSparkMax(2, MotorType.kBrushless);
    encoder = motor.getEncoder();
    motor.setInverted(false);
    motor2.follow(motor, true);
    
    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    pidController = motor.getPIDController();
  
    /**
     * The PID Controller can be configured to use the analog sensor as its feedback
     * device with the method SetFeedbackDevice() and passing the PID Controller
     * the CANAnalog object. 
     */
    pidController.setFeedbackDevice(encoder);

    // PID coefficients
    kP = 0.0001; 
    kI = 0.0;
    kD = 0.0; 
    kIz = 0.0; 
    kFF = 0.0021; 
    kMaxOutput = 1.0; 
    kMinOutput = -1.0;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    //motor.pidWrite(output);

    encoder.setVelocityConversionFactor(1);

    robotContainer = new RobotContainer();
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
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    
  }

  Double max = 0.0;
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // target = (stick.getRawAxis(2) - 1) * -0.5;
    // motor.set(target);
    target = ((stick.getRawAxis(2) - 1) * -0.5) * 5000;
    //target = (stick.getRawAxis(2) - 1) * -0.5;
      // error = encoder.getVelocity() - target;
      // pidController.setFF(0.0021 / averager(motor.getBusVoltage()));
      // pidController.setReference(target, ControlType.kVelocity);
    //motor.set(target);
    if (encoder.getVelocity() < target) {
      motor.set(1);
    } else {
      motor.set(0);
    }
    addItem(error);
    
    SmartDashboard.putNumber("Target", target);
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Ratio", (motor.getBusVoltage() * motor.getAppliedOutput()) / encoder.getVelocity());
    SmartDashboard.putNumber("CurrentLeader", motor.getOutputCurrent());
    SmartDashboard.putNumber("Voltage", motor.getAppliedOutput() * motor.getBusVoltage());
    SmartDashboard.putNumber("Error", encoder.getVelocity() - target);
    SmartDashboard.putNumber("FF", pidController.getFF());
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

  public void addItem(double newVal){
    double temp1 = errorVals[0];
    double temp2;
    errorVals[0] = newVal;
    for (int i = 1; i < errorVals.length; i++) {
      temp2 = errorVals[i];
      errorVals[i] = temp1;
      temp1 = temp2;
    }
  }
  public double averager (double newVal) {
    double[] voltages = new double[3];
    int trueLength = 0;
    double total = 0;
    voltages[counter % voltages.length] = newVal;
    counter++;
    for (double i:voltages) {
      if (i != 0.0) {
        total += i;
        trueLength++;
      } 
      counter++;
    } if (trueLength > 0) {
      return total / trueLength;
    } else {
      return 12.5;
    }
  }
}
