// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin. */
private static Spark m_blinkin= null;

  public Blinkin(int pwmPort){

    m_blinkin = new Spark(0);
    solidBlack();
  
  
}

public void fire(){
  m_blinkin.set(-0.59);
}

public void solidBlack(){
  m_blinkin.set(-0.73);

}

public void breathBlue(){
m_blinkin.set(-0.15);

}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


public void allianceColor(){
boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
if (isRed == true){
  m_blinkin.set(-0.39);
  System.out.println("Color Waves, Lava Palette");
} else{
  m_blinkin.set(-0.23);
  System.out.println("Heartbeat, Blue");

}
}

}
