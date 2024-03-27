package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drive;
import frc.robot.pathfinding.CharliesAstar;
import frc.robot.pathfinding.GriddedField;
import frc.robot.pathfinding.Path;
import java.util.List;

public class PathFollowing {

  // How many periods before the robot updates the field and calculates a new path.
  // Increasing this number improves performance but also lengthens reaction time.
  private final int REFRESH_INTERVAL = 5;

  private int refreshMeter = 5;

  private Path path;
  private Drive drive;
  private GriddedField field;
  private CharliesAstar aStar;

  public PathFollowing(Path path, Drive drive, GriddedField field) {
    this.path = path;
    this.drive = drive;
    this.field = field;
    aStar =
        new CharliesAstar(
            field,
            field.coordsToBox(new Translation2d(path.endingPoint.getX(), path.endingPoint.getY())));
  }

  /**
   * Checks whether the robot is at its goal, and if it is, starts the robot onto another goal.
   *
   * @param newGoal The next goal pose, if the robot reaches its current goal.
   */
  public void pathEndCheck(Pose2d newGoal) {
    if (path.atEndPoint()) {
      newPath(newGoal);
    }
  }

  /**
   * Sets the current path to a new one with a different goal.
   *
   * @param goal The new goal of the path.
   */
  public void newPath(Pose2d goal) {
    path = new Path(drive.getPose(), goal, field, drive);
  }

  /**
   * Sends the robot to drive to the next point on its path toward the goal. Also finds the next
   * pose.
   *
   * @param speed The speed wanted at the next pose.
   * @return A command to send the robot to the next position in its heroic journey.
   */
  public Command goToNextPose(double speed) {
    field.addTempObstacles(field.getMovingObstacles());
    if (refreshMeter >= REFRESH_INTERVAL) {
      aStar.assignCosts(field.coordsToBox(path.endingPoint.getTranslation()));
      refreshMeter = 0;
    } else {
      refreshMeter++;
    }
    return drive.goToState(
        new Pose2d(aStar.nextPos(drive.getPose().getTranslation()), path.endingPoint.getRotation()),
        speed);
  }

  /**
   * Calculates a path from the starting point to the ending point which avoids all obstacles.
   *
   * @return A list of Translation2ds which represent the coordinate path of the robot.
   */
  public List<Translation2d> calculatePath() {
    field.addTempObstacles(field.getMovingObstacles());
    return aStar.pathMaker(drive.getPose().getTranslation(), path.endingPoint.getTranslation());
  }

  /**
   * Calculates the next pose and the suggested speed to the next pose using drive's PID controller,
   * then goes to it.
   *
   * @return A command moving the robot to the next pose.
   */
  public Command goToNextPose() {
    return goToNextPose(
        Math.hypot(
            drive.pid.calculate(drive.getPose().getX(), path.endingPoint.getX()),
            drive.pid.calculate(drive.getPose().getY(), path.endingPoint.getY())));
  }
}
