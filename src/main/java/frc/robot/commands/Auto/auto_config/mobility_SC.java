
package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;
import frc.robot.commands.Auto.Cmd.ResetSensors;

public class mobility_SC extends SequentialCommandGroup {

  private static final Trajectory mobility_sin_cables = PathPlanner.loadPath("mobility_sin_cables", AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared,true);

  public mobility_SC() {

    addCommands(
        new ParallelDeadlineGroup(new WaitCommand(0.1), new SequentialCommandGroup(new ResetSensors(RobotContainer.driveSubsystem))),
        new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.driveSubsystem,mobility_sin_cables).robotRelative())

        );

  }
}
