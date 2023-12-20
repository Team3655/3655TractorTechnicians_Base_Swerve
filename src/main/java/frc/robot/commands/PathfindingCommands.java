package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfindingCommands {

  // If odometry error is accumulating during pathfinding reduce accelerations
  // (this is the number one culprit of odometry loss)

  public static final double MAX_VELOCITY = 3.0;
  public static final double MAX_ACCELERATION = 4.0;
  public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540);
  public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(540);

  // Create the constraints to use while pathfinding
  private static final PathConstraints constraints =
      new PathConstraints(
          MAX_VELOCITY, MAX_ACCELERATION, MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);

  /**
   * Generates a new PathPlanner pathfinding command targeting a position. If the robot is currently
   * in motion when the command is run the robot will "curve" the path to "ease out" of its current
   * speed to fufill the acceleration constraint.
   *
   * @param targetPose The target pose of the pathfinding command (must be within the valid area in
   *     the PP navgrid)
   * @param endVelocity The goal end velocity of the robot when reaching the target pose (usually
   *     zero but set higher if you don't want the robot to slow down when it reaches its target)
   * @param rotationDelay The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation (in meters)
   * @return
   * @see https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Automatic-Pathfinding
   */
  public static Command pathfindToPose(
      Pose2d targetPose, double endVelocity, double rotationDelay) {

    // See the "Follow a single path" example for more info on what gets passed here
    Command pathfindingCommand =
        AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            // Goal end velocity in meters/sec
            endVelocity,
            // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
            rotationDelay);

    return pathfindingCommand;
  }

  /**
   * Generates a new PathPlanner pathfinding command targeting the start of a path, then following
   * the path. If the robot is currently in motion when the command is run the robot will "curve"
   * the path to "ease out" of its current speed to fufill the acceleration constraint.
   *
   * @param pathName The name of the path file to follow (excluding the .path filename extension)
   * @param rotationDelay The distance the robot should move from the start position before
   *     attempting to rotate to the final rotation (in meters)
   * @return
   * @see https://github.com/mjansen4857/pathplanner/wiki/Java-Example:-Automatic-Pathfinding
   */
  public static Command pathfindToPath(String pathName, double rotationDelay) {

    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    // See the "Follow a single path" example on PP wiki for more info on what gets
    // passed here
    Command pathfindingCommand =
        AutoBuilder.pathfindThenFollowPath(path, constraints, rotationDelay);

    return pathfindingCommand;
  }
}
