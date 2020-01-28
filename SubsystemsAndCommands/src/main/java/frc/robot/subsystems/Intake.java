/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.IntakeC;
import frc.robot.subsysconsts.*;

public class Intake extends SubsystemBase {

  /**
   * Creates a new ExampleSubsystem.
   */
  public Intake() {

  }
  
  public void takeIn(){
    IntakeShooterConst.intakeMotor.set(Constants.talonCM, IntakeShooterConst.intakeSpeed);
  }

  public void takeOut(){
    IntakeShooterConst.intakeMotor.set(Constants.talonCM, -IntakeShooterConst.intakeSpeed);
  }

  public void endMotors(){
    IntakeShooterConst.intakeMotor.set(Constants.talonCM, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
