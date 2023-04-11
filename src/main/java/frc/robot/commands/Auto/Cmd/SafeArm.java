// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.posicionesbrazo.posicionSafeArm;

public class SafeArm extends CommandBase {
  /** Creates a new SafeArm. */
  public SafeArm() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.brazosubsystem.movimiento_brazo_angulo(posicionSafeArm.ejeinferior, posicionSafeArm.ejesuperior);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.brazosubsystem.ejeinferior(0);
    RobotContainer.brazosubsystem.ejesuperior(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
