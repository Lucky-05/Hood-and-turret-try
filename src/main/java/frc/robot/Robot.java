// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.ChassisSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_DriveStraight;
  private Command m_ExampleTrajectory;
  public final static ChassisSubsystem driveTrain = new ChassisSubsystem();
  private RobotContainer m_robotContainer;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      m_DriveStraight.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_DriveStraight.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public Command getAutonomousCommand(){
    TrajectoryConfig config = new TrajectoryConfig(10, 5);
    config.setKinematics(driveTrain.getKinematics());
    Trajectory exampleTrajectory =  TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d(0)), List.of(new Translation2d(1,1), new Translation2d(2,-1)),new Pose2d(3.0,0, new Rotation2d(0)), config);

//RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, driveTrain::getPose, new RamseteController(ChassisConstants.ramseteB, ChassisConstants.ramseteZeta), new SimpleMotorFeedforward(ChassisConstants.ks, ChassisConstants.kv), ChassisConstants.kinematics, driveTrain.getspeeds(), new PIDController(ChassisConstants.kp, 0, 0), new PIDController(ChassisConstants.kp, 0, 0), driveTrain::tankDrive, driveTrain);

RamseteCommand ramseteCommand =
new RamseteCommand(
    exampleTrajectory,
    driveTrain::getPose,
    new RamseteController(ChassisConstants.ramseteB, ChassisConstants.ramseteZeta),
    new SimpleMotorFeedforward(
        ChassisConstants.ks,
        ChassisConstants.kv,
        ChassisConstants.ka),
    driveTrain.getKinematics(),
    driveTrain::getspeeds,
    new PIDController(ChassisConstants.kp, 0, 0),
    new PIDController(ChassisConstants.kp, 0, 0),
    // RamseteCommand passes volts to the callback
    driveTrain::tankDrive,
    driveTrain);

    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    return ramseteCommand.andThen(()-> driveTrain.tankDrive(0, 0));
  }
}

