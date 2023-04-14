
package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Utils.TrajectoryLoader;
import frc.robot.subsystems.DriveSubsystem;

public class DrivetrainRamseteCommand extends RamseteCommand {

    protected boolean resetPosition;
    protected Trajectory trajectory;
    protected DriveSubsystem driveSubsystem;

    public DrivetrainRamseteCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        super(
                trajectory,
                driveSubsystem::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                driveSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                new PIDController(DriveConstants.kPDriveVel, 0, 0),

                driveSubsystem::tankDriveVolts,
                driveSubsystem);

        this.resetPosition = true;
        this.trajectory = trajectory;
        this.driveSubsystem = driveSubsystem;

    }

    public DrivetrainRamseteCommand(DriveSubsystem driveSubsystem, String path) {
        this(driveSubsystem, TrajectoryLoader.getTrajectory(path));
    }

    public DrivetrainRamseteCommand(DriveSubsystem driveSubsystem, String... paths) {
        this(driveSubsystem, TrajectoryLoader.getTrajectory(paths));
    }

    public DrivetrainRamseteCommand robotRelative() {
        this.resetPosition = true;
        return this;
    }

    public DrivetrainRamseteCommand fieldRelative() {
        this.resetPosition = false;
        return this;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (resetPosition) {
            driveSubsystem.resetOdometry(trajectory.getInitialPose());
        }
    }
}
