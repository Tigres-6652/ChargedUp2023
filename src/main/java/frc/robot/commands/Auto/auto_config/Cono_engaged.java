// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;
import frc.robot.commands.Auto.Cmd.ResetSensors;
import frc.robot.commands.Auto.Cmd.MantenerGP;
import frc.robot.commands.Auto.Cmd.soltarcono;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cono_engaged extends SequentialCommandGroup {

  private static final Trajectory Engaged = PathPlanner.loadPath("Engaged", AutoConstants.kMaxSpeedMetersPerSecond,
  AutoConstants.kMaxAccelerationMetersPerSecondSquared,true);
  
  public Cono_engaged() {

    
    addCommands(    
      new ParallelDeadlineGroup(new WaitCommand(1.2),new SequentialCommandGroup(new BrazoCmd(RobotContainer.brazosubsystem, () -> 0.0, () -> 0.0, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false)),new MantenerGP()),
      new ParallelDeadlineGroup(new WaitCommand(1.2),new SequentialCommandGroup(new BrazoCmd(RobotContainer.brazosubsystem, () -> 0.0, () -> 0.0, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false)),new MantenerGP()),
      new ParallelDeadlineGroup(new WaitCommand(0.9),new SequentialCommandGroup(new BrazoCmd(RobotContainer.brazosubsystem, () -> 0.0, () -> 0.0, ()->false, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false)),new MantenerGP()),
      new ParallelDeadlineGroup(new WaitCommand(0.4) , new SequentialCommandGroup(new soltarcono())),
      new ParallelDeadlineGroup(new WaitCommand(1),new SequentialCommandGroup(new BrazoCmd(RobotContainer.brazosubsystem,   () -> 0.0, () -> 0.0, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false))),
      new ParallelDeadlineGroup(new WaitCommand(0.1), new SequentialCommandGroup(new ResetSensors(RobotContainer.driveSubsystem))),
      new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.driveSubsystem,Engaged).robotRelative()),
      new ParallelDeadlineGroup(new WaitCommand(10),new SequentialCommandGroup(new DriveTrainCmd(RobotContainer.driveSubsystem, ()->0.0, ()->0.0, ()->false, ()->true, ()->false, ()->true)))
  
);
  }
}
