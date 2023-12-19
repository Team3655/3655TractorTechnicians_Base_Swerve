package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfindingCommands {

  public static final double MAX_VELOCITY = 3.0;
  public static final double MAX_ACCELERATION = 4.0;
  public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540);
  public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(720);

  // Create the constraints to use while pathfinding
  private static final PathConstraints constraints =
      new PathConstraints(
          MAX_VELOCITY, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);

  // Since we are using a holonomic drivetrain, the rotation component of this
  // pose represents the goal holonomic rotation
  public static Command pathfindToPose(Pose2d targetPose) {

    // See the "Follow a single path" example for more info on what gets passed here
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
            );

    return pathfindingCommand;
  }

  public static Command pathfindToPath(String pathName) {

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // See the "Follow a single path" example for more info on what gets passed here
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, constraints, 0);

    return pathfindingCommand;
  }
}
