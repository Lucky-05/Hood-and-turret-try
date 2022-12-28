// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new climber. */
  private Solenoid climberIn = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.climberIn);
  private Solenoid climberOut = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.climberOut);

  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  boolean enabled = pcmCompressor.enabled();
  boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
  double current = pcmCompressor.getCurrent();

  public ClimberSubsystem() {}

  public void toggleClimber(){
    boolean climberstate = climberIn.get();
    if(climberstate){

      climberIn.set(true);
      climberOut.set(false);
    }else{
      climberIn.set(false);
      climberOut.set(true);
    }

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
