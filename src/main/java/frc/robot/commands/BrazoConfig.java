// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BrazoSubsystem;

public class BrazoConfig extends CommandBase {

  private Supplier<Double> GradosInferior, GradosSuperior;
  private final BrazoSubsystem brazosubsystem;
  double gradosejeinf;
  double gradosejesup;

  public BrazoConfig(BrazoSubsystem brazosubsystem) {
    
    this.brazosubsystem = brazosubsystem;

    addRequirements(brazosubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    brazosubsystem.config_motor_eje_inf();

    SmartDashboard.putNumber("get inf", gradosejeinf);
    SmartDashboard.putNumber("get sup", gradosejesup);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.getNumber("get inf", gradosejeinf);
    SmartDashboard.getNumber("get sup", gradosejesup);

    brazosubsystem.movimiento_brazo_angulo(gradosejeinf, gradosejesup);/// dejar

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
