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
  CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
  CANPIDController pidController = new CANPIDController(motor1);  //Possible Error
  CANEncoder encoder = new CANEncoder(motor1);

  double kP;
  double kI;
  double kD;
  double kF;
  double kIz;
  double kMaxOutput;
  double kMinOutput;

  double P;
  double I;
  double D;
  double F;
  double Iz;
  double Max;
  double Min;
  double InTarget;

  double targetSpeed;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor1.setIdleMode(IdleMode.kCoast);
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
    P = SmartDashboard.getNumber("P Gain", 0);
    I = SmartDashboard.getNumber("I Gain", 0);
    D = SmartDashboard.getNumber("D Gain", 0);
    Iz = SmartDashboard.getNumber("I Zone", 0);
    F = SmartDashboard.getNumber("Feed Forward", 0);
    Max = SmartDashboard.getNumber("Max Output", 0);
    Min = SmartDashboard.getNumber("Min Output", 0);
    InTarget = SmartDashboard.getNumber("Target", 0);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Position", encoder.getPosition());

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
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Target", targetSpeed);
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

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((P != kP)) { pidController.setP(P); kP = P; }
    if((I != kI)) { pidController.setI(I); kI = I; }
    if((D != kD)) { pidController.setD(D); kD = D; }
    if((Iz != kIz)) { pidController.setIZone(Iz); kIz = Iz; }
    if((F != kF)) { pidController.setFF(F / motor1.getBusVoltage()); kF = F; }
    if((InTarget != targetSpeed)) { targetSpeed = InTarget; }
    if((Max != kMaxOutput) || (Min != kMinOutput)) { 
      pidController.setOutputRange(Min, Max);
      kMinOutput = Min; kMaxOutput = Max;
    }

    SmartDashboard.putNumber("Velocity", encoder.getVelocity());
    SmartDashboard.putNumber("Applied Percent", motor1.getAppliedOutput());
    pidController.setFF(F / motor1.getBusVoltage()); kF = F;
    pidController.setReference(targetSpeed, ControlType.kVelocity);
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
