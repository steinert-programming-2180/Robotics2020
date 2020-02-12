/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ExampleSubsystem;

public class MoveToAngle extends PIDCommand {
  /**
   * Creates a new MoveToAngle.
   */
  double kTurnP = 0;
  double kTurnI = 0;
  double kTurnD = 0;
  public MoveToAngle(double deltaAngle, ExampleSubsystem ex, AHRS ahrs, TalonSRX[] leftMotors, TalonSRX[] rightMotors) {
    super(
        new PIDController(kTurnP, kTurnI, kTurnD),
        // Close loop on heading
        ahrs.getAngle(),
        // Set reference to target
        deltaAngle+ahrs.getAngle(),
        // Pipe output to turn robot
        output -> function(output, leftMotors, rightMotors)
        // Require the drive
        );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void function(double speed, TalonSRX[] leftMotors, TalonSRX[] rightMotors){
    for(TalonSRX i : leftMotors){
      i.set(ControlMode.PercentOutput, speed);
    }

    for(TalonSRX i : rightMotors){
      i.set(ControlMode.PercentOutput, -speed);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //Reassigns angles so reported angle is between -180 and 180, and target is 0
  public double correctToBoundedAngle(double targetAngle, double currentAngle) {
    return 0.0; 
  }
}
