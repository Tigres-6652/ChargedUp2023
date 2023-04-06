// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.GarraCmd;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ONE_GP_balanceo extends SequentialCommandGroup {

DriveSubsystem driveSubsystem=new DriveSubsystem();

private static final Trajectory chargestation = PathPlanner.loadPath("ONE_GP_balanceo", Constants.AutoConstants.kMaxSpeedMetersPerSecond , Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);


  public ONE_GP_balanceo() {

    addCommands(

    new ParallelDeadlineGroup(new BrazoCmd(RobotContainer.brazosubsystem, ()->0.0, ()->0.0, ()->false, ()->false, ()->false, ()->false, ()->true), new WaitCommand(4)),
    new ParallelDeadlineGroup(new GarraCmd(RobotContainer.garraSusbsytem, ()->true, ()->false), new WaitCommand(1)),
    new ParallelDeadlineGroup(new BrazoCmd(RobotContainer.brazosubsystem, ()->0.0, ()->0.0, ()->false, ()->true, ()->false, ()->false, ()->true)),
    new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.driveSubsystem, chargestation) .robotRelative())
   // ,new ParallelDeadlineGroup(new DriveTrainCmd(driveSubsystem, ()->0.0, ()->0.0, ()->false, ()->true, ()->false))

    );
  }
}
