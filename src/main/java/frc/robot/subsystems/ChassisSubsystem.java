// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import javax.swing.text.TabExpander;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ChassisSubsystem extends SubsystemBase {
  /** Creates a new ChassisSubsystem. */
  
  private CANSparkMax frontRight = new CANSparkMax(ChassisConstants.frontRight, MotorType.kBrushed);
  private CANSparkMax frontLeft = new CANSparkMax(ChassisConstants.frontLeft, MotorType.kBrushed);
  private CANSparkMax backRight = new CANSparkMax(ChassisConstants.backRight, MotorType.kBrushed);
  private CANSparkMax backLeft = new CANSparkMax(ChassisConstants.backLeft, MotorType.kBrushed);
  PhotonCamera camera = new PhotonCamera("photonvision");  //UI name
  PhotonPipelineResult result = camera.getLatestResult();
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");


  NetworkTableEntry horizontalOffset = limelightTable.getEntry("tx");
  NetworkTableEntry verticalOffset = limelightTable.getEntry("ty");
  NetworkTableEntry targetArea = limelightTable.getEntry("ta");
 double kp= -0.1;
  double tx= horizontalOffset.getDouble(0.0);   //Default cvalue in case it doesnt exist one that is valid
  double y= verticalOffset.getDouble(0.0);
  double a= targetArea.getDouble(0.0);





  
  public DifferentialDrive chasis;

  

  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(ChassisConstants.kinematics); //27 inch
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ChassisConstants.ks, Constants.ChassisConstants.kv, Constants.ChassisConstants.ka);



  public static final double maxSpeed = 4.15; //In m/s
  public static final double maxAcceleration = 3; //In m/S squared 
  public static final double ramseteB = 2;    //Both ramsete cconstants are used to control the output voltage of the motors 
  public static final double ramseteZeta = 0.7; 

  PIDController leftPidController = new PIDController(9.95, 0, 0);
  PIDController righPidController = new PIDController(9.95, 0, 0);

  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private double LeftSpeed,RightSpeed;

  public double angle;
  Pose2d pose;

    

  
  public ChassisSubsystem(){
    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    frontLeft.setInverted(true);
    frontRight.setInverted(false);
    backLeft.follow(frontLeft, true);
    backRight.follow(frontRight, false);
  chasis = new DifferentialDrive(frontLeft, frontRight);   
  rightEncoder = frontRight.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
  leftEncoder = frontLeft.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
                                                                   // enable,limit amperes, threshholdm, timeout 
  


  }

  public void resetAHRSGyro(){
    gyro.reset();
  }

  public DifferentialDriveKinematics getKinematics(){
return kinematics;

  }
  
public SimpleMotorFeedforward gFeedforward(){
  return feedforward;
}

public Pose2d getPose(){
  return pose;
}
  public Rotation2d getHeading(){
return Rotation2d.fromDegrees(-gyro.getAngle());

  }

  public PIDController getRighPidController(){
    return righPidController;
  }
  public PIDController geTleftPidController(){
    return leftPidController;
  }

  public double getLeftSpeeds(){
    return rightEncoder.getVelocity()/60*2*Math.PI*Units.inchesToMeters(6)/60;  //the double need to be in m/s 
  }

  public double getRightSpeeds(){
return rightEncoder.getVelocity()/60*2*Math.PI*Units.inchesToMeters(6)/60; // the double needs to be in m/s

  }

  public DifferentialDriveWheelSpeeds getspeeds(){
    return new DifferentialDriveWheelSpeeds(
       leftEncoder.getVelocity()/60*2*Math.PI*Units.inchesToMeters(6)/60,
       rightEncoder.getVelocity()/60*2*Math.PI*Units.inchesToMeters(6)/60
    );

  }

  public Pose2d resetOdometry(Pose2d pose2d){

    return pose2d;
  } 
  

  public void TankDrive(double leftSpeed, double rightSpeed){
    LeftSpeed = leftSpeed;
    RightSpeed = rightSpeed;
    chasis.tankDrive(LeftSpeed, RightSpeed);

  }
  
  public void publishData(){
    
  }

  public void tankDrive(double leftVolts, double rightVolts){
    frontRight.setVoltage(rightVolts);
    frontLeft.setVoltage(rightVolts);
    chasis.feed();                        //Se usa para evitar caidas en el voltaje y maximizar la energia!
  } 

  public void aims(){
    Double adjust= 0.0;
    Double headingError = -tx;
    Double minCommand= 0.05;
    
    if(tx > 1.0){
      adjust= kp*headingError-minCommand;
    }else if(tx< 1.0){
      adjust=kp*headingError+minCommand;
      
    }
    LeftSpeed+=adjust;
    RightSpeed-=adjust;

    
  }

 
  @Override
  public void periodic() {
  publishData();
    // This method will be called once per scheduler run
    pose = odometry.update(getHeading(), getLeftSpeeds(),getRightSpeeds());
    if(result.hasTargets()){
      var target = result.getBestTarget();
      var yaw = target.getYaw();
      var pitch = target.getPitch();
      var camTotarget= target.getBestCameraToTarget(); 
                         
    }
  }
}
