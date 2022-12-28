// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;


public class ChassisTankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final ChassisSubsystem chassis;
 private final double Left, Right;
  public ChassisTankDrive(ChassisSubsystem chasis, double left, double right) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Left = left;
    this.Right = right;
    this.chassis = chasis;

    addRequirements(chassis);
  }

 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    chassis.TankDrive(Left, Right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
