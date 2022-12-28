// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{
public final class ChassisConstants{
    public static final int frontRight = 1; 
    public static final int frontLeft = 5; 
    public static final int backRight = 4; 
    public static final int backLeft = 3; 
    
    public static final double ka = 0.43350;
    public static final double kv = 2.4020;
    public static final double ks = 0.52270;
    public static final double kinematics = 0.6858;

    public static final double maxSpeed = 3; //In m/s
    public static final double maxAcceleration = 1;

    public static final double ramseteB = 2;    //Both ramsete cconstants are used to control the output voltage of the motors 
    public static final double ramseteZeta = 0.7;

    public static final double kp = 9.95;
}

public final class turretConstants{
    public static final int neo550Id = 10;
    public static final int pwmPortSwitch0 =0;
    public static final int pwmPortSwitch1 =1;

}
public final class OiConstnats{
public static final int ControllerInput= 0; 

}

public final class HoodConstants{
    public static final int motor0Id = 12;
    public static final int motor1Id = 13;
    public static final int angleMotorId = 14; 
    public static final double anglePerMeter = 4.88;
    public static final double robotHeight =1.5;
    public static final double targetHeight = 2.35;
    public static final double cameraAngle = 40.0; 
    public static final double rpmPerMeter = 6.0;
    public static final double minHoodAngle = 20.03; // 1 meter
    public static final double minRpm = 4194; // 1 meter 
    public static final double shooterkP = 5e-5;
    public static final double shooterkI = 1e-7;
    public static final double shooterkD = 1e-3;
    public static final double shooterkIz = 0;
    public static final double shooterkFF = 1.5e-4;
    public static final double ksVolts = 0.54039;
    public static final double kvVolts = 0.12848;
    
}

public final class ClimberConstants{

    public static final int climberIn = 1;
    public static final int climberOut = 2;

}
public final class LedConstants{
public static final int pwmPort= 0;

}
}
