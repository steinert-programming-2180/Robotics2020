/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    final int controllerPort = 0;
    final int leftJoystick = 1;
    final int rightJoystick = 5;

    final int[] leftPorts = {4, 5, 6};
    final int[] rightPorts = {1, 2, 3};

    TalonSRX[] leftTals = new TalonSRX[3];
    TalonSRX[] rightTals = new TalonSRX[3];
    TalonSRX[] allTals = new TalonSRX[6];

    final XboxController controller = new XboxController(0);
}
