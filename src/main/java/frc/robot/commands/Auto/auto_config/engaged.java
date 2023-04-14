
package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;

public class engaged extends SequentialCommandGroup {

  private static final Trajectory Engaged = PathPlanner.loadPath("Engaged", AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  public engaged() {

    addCommands(

        new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.driveSubsystem,Engaged).robotRelative()),
        new ParallelDeadlineGroup(new WaitCommand(4),new SequentialCommandGroup(new DriveTrainCmd(RobotContainer.driveSubsystem, ()->0.0, ()->0.0, ()->false, ()->true, ()->false, ()->true)))

        );

  }
}
