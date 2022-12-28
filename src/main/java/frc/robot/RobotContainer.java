// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.OiConstnats;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ChassisSubsystem chassis = new ChassisSubsystem();
  private final XboxController joystick1 = new XboxController(OiConstnats.ControllerInput);
 private final ClimberSubsystem climber = new ClimberSubsystem();
 private final Blinkin m_blinkin = new Blinkin(LedConstants.pwmPort);
  String trajectoryJsonString = "file:///C:/Users/lucia/OneDrive/Escritorio/PathWeaver/Paths/Path%20de%203%20pelotas";
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    chassis.setDefaultCommand(
      new RunCommand(() -> chassis.TankDrive(joystick1.getLeftY(),joystick1.getRightY()), chassis));
      
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton b = new JoystickButton(joystick1, Button.kB.value);
b.whileHeld(()-> chassis.aims(), chassis);
  final JoystickButton aButton = new JoystickButton(joystick1, Button.kA.value);
  aButton.whileHeld(() -> m_blinkin.fire(), m_blinkin); 
  aButton.whenReleased(() -> m_blinkin.breathBlue(), m_blinkin);
    new JoystickButton(joystick1, Button.kY.value)
    //.whenPressed(new InstantCommand(()-> climber.toggleClimber()));
    .whenPressed(new InstantCommand(() -> climber.toggleClimber()));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods

}

