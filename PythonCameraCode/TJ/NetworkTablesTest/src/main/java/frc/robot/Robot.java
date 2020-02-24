/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  double x = 0;
  double y = 0;
 
  public void robotInit() {
    //Get the default instance of NetworkTables that was created automatically
    //when your program starts
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Get the table within that instance that contains the data. There can
    // be as many tables as you like and exist to make it easier to organize
    // your data. In this case, it's a table called datatable.
    final NetworkTable table = inst.getTable("datatable");

      //Get the entries within that table that correspond to the X and Y values
      //for some operation in your program.
      xEntry = table.getEntry("X");
      yEntry = table.getEntry("Y");
    }

    public void teleopPeriodic() {
      //Using the entry objects, set the value to a double that is constantly
      //increasing. The keys are actually "/datatable/X" and "/datatable/Y".
      //If they don't already exist, the key/value pair is added.
      xEntry.setDouble(x);
      yEntry.setDouble(y);
      x += 0.05;
      y += 1.0;
    }
  }