// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.turretConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  double tx= limelight.getEntry("tx").getDouble(0.0);
  double ty = limelight.getEntry("ty").getDouble(0.0);
  double tv =limelight.getEntry("tv").getDouble(0.0);
  private  CANSparkMax neo550 = new CANSparkMax(turretConstants.neo550Id, MotorType.kBrushless);
  //private RelativeEncoder encoder= neo550.getAlternateEncoder(Type.kQuadrature, 42);
 // private int countPerAngle = 10;
  private double kp = -0.1;
  double speed=0.0; 
  double adjust= 0.0;
  double headingError = -tx;
  double minCommand= 0.05;
  double searchSpeed = 0.5;
  private DigitalInput rightSwitch = new DigitalInput(turretConstants.pwmPortSwitch0);
  private DigitalInput leftSwitch = new DigitalInput(turretConstants.pwmPortSwitch1);
  
  public Turret() {
  neo550.setIdleMode(IdleMode.kBrake);
  }
  public void setSpeed(double speed){
    neo550.set(speed);
  }
  public void turretStop(){
    neo550.set(0);
  }
  public double getTx(){
    return tx;
  }
  public double getTv(){
    return tv;
  }
  public void aiming(){
    if(tx > 1.0){
      adjust= kp*headingError-minCommand;
      setSpeed(speed+=adjust);
    }else if(tx< 1.0){
      adjust=kp*headingError+minCommand;
      setSpeed(speed -=adjust);
      
    }
    
  }

  public void searchingTarget(){
    if(rightSwitch.get()) setSpeed(-searchSpeed);
    else if(leftSwitch.get()) setSpeed(searchSpeed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
   
    
  }


  
}
