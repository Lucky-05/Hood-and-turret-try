// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.Hood;

public class HoodCommand extends CommandBase {
  /** Creates a new HoodCommand. */

  Hood hood; 
  double ty = hood.getTy();
  double distance = hood.getdistance(ty);
  public HoodCommand(Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hood);
    this.hood = hood;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setAngle(distance*HoodConstants.anglePerMeter);
    if(hood.getTV() != 1){
      hood.shooterRest();
    } else if(hood.getTV() ==1){
      updateSetpoint();
    }
    
  }
  public void updateSetpoint(){ //needs to be in rps 
    hood.setSetpoint((HoodConstants.minRpm+(distance*HoodConstants.rpmPerMeter))/60);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
