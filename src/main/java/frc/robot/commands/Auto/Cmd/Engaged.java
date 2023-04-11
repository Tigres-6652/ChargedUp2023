// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Engaged extends CommandBase {
  /** Creates a new Engaged. */

  public Engaged() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  @Override
  public void execute() {

    RobotContainer.driveSubsystem.CHASIS(0, 0, false, true, false, false);

  }


  @Override
  public void end(boolean interrupted) {    
    RobotContainer.driveSubsystem.CHASIS(0, 0, false, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
