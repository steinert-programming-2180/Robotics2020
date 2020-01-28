package frc.robot.subsysconsts;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeShooterConst{
    public static int ballsAllowed;
    public static int currentBalls = 0;

    public static TalonSRX intakeMotor = new TalonSRX(1);
    public static double intakeSpeed = .5;

    public static boolean isReversed = false;
}