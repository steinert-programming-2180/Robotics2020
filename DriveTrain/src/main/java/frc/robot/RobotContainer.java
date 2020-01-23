/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  TalonSRX talL0;
  TalonSRX talL1;
  TalonSRX talL2;
  TalonSRX talR0;
  TalonSRX talR1;
  XboxController controller;
  double speedFactor = 1.0/3.0;

  controller = new XboxController(0);

  talL0 = new TalonSRX(6);
  talL1 = new TalonSRX(10);
  talL2 = new TalonSRX(4);
  talR0 = new TalonSRX(9);
  talR1 = new TalonSRX(2);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    double leftJoystickAxis = controller.getRawAxis(1);
    double rightJoystickAxis = controller.getRawAxis(5);
    double rTrigger = controller.getRawAxis(3);

    if(rTrigger != 0){
      speedFactor = rTrigger;
      SmartDashboard.putNumber("TriggerVl", rTrigger);
    } else{
      SmartDashboard.putNumber("TriggerVl", rTrigger);
      speedFactor = 1.0/3.0;
    }

    talL0.set(ControlMode.PercentOutput, leftJoystickAxis*speedFactor);
    talL1.set(ControlMode.PercentOutput, leftJoystickAxis*speedFactor);
    talL2.set(ControlMode.PercentOutput, leftJoystickAxis*speedFactor);
    talR0.set(ControlMode.PercentOutput, -rightJoystickAxis*speedFactor);
    talR1.set(ControlMode.PercentOutput, -rightJoystickAxis*speedFactor);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
