// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  private CANSparkMax  motor0 = new CANSparkMax( Constants.HoodConstants.motor0Id,MotorType.kBrushless);
  private CANSparkMax motor1 = new CANSparkMax(HoodConstants.motor1Id, MotorType.kBrushless);
  private CANSparkMax angleMotor = new CANSparkMax(HoodConstants.angleMotorId, MotorType.kBrushless);
  //private SparkMaxPIDController shooterPID; 
  private double encoderCounts = 4096;
  private RelativeEncoder encoder;
  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private double ty = limelight.getEntry("ty").getDouble(0.0);
  private PIDController m_Controller = new PIDController(HoodConstants.shooterkP, HoodConstants.shooterkI, HoodConstants.shooterkD);
  private final SimpleMotorFeedforward shooterFeed = new SimpleMotorFeedforward(HoodConstants.ksVolts, HoodConstants.kvVolts);
  
   double distance = 0.0; 
  

  

 
  public double getTy(){
      return ty;
  }
  public void useOutput(double output, double setpoint){
    motor0.setVoltage(output+shooterFeed.calculate(setpoint));
  }
  public void setSpeedShooter(double speed){
    motor0.set(speed);
  }

  public void setAngle(double encoderCounts){
    encoder.setPosition(encoderCounts);
  }

  public void shooterRest(){
    motor0.set(0.25);
  }

  

  public double getMeasurment(){  // in rpm 
    double velocity = encoder.getVelocity();
    double rotPerSec = velocity/encoderCounts*10;
    return  rotPerSec*60;
  }

  public boolean atSetpoint(){
    return m_Controller.atSetpoint();
  }

  public double getdistance(double Ty){
   return (HoodConstants.targetHeight-HoodConstants.robotHeight)/(Math.tan(Math.toRadians(Ty+HoodConstants.cameraAngle))); 
  }

  public Hood() {

encoder = angleMotor.getAlternateEncoder(Type.kQuadrature, 4096);
motor0.setInverted(false);
motor1.follow(motor0, false);
motor0.setIdleMode(IdleMode.kCoast);
motor1.setIdleMode(IdleMode.kCoast);
angleMotor.setIdleMode(IdleMode.kBrake);



  }
public void setSetpoint(double setpoint){
   m_Controller.setSetpoint(setpoint);
 
}
public void disableShooter(){
  motor0.set(0);
  angleMotor.set(0);
}

public double getTV(){
  return limelight.getEntry("tv").getDouble(0.0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distance = getdistance(ty);

  }
}
