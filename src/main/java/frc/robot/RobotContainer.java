// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Joystick joystick = new Joystick(0);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(new ChassisSpeeds(
      Constants.constraints.maxVelocity().times(joystick.getRawAxis(1)), 
      Constants.constraints.maxVelocity().times(joystick.getRawAxis(0)), 
      Constants.constraints.maxAngularVelocity().times(joystick.getRawAxis(2))), 
      false, false));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
