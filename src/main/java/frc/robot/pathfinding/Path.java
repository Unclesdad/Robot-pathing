package frc.robot.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class Path {
  public final Pose2d startingPoint;
  public final Pose2d endingPoint;

  public final List<Obstacle> obstacles;

  public static final AffineTransform nullTransform = new AffineTransform();

  public Path(Pose2d startingPoint, Pose2d endingPoint, List<Obstacle> obstacles) {
    this.startingPoint = startingPoint;
    this.endingPoint = endingPoint;
    this.obstacles = obstacles;
  }

  /**
   * Static method to find out whether a point can be driven straight to from another point.
   *
   * @param startPoint The starting point.
   * @param setPoint The ending point.
   * @return whether the ending point can be driven straight to from the starting point.
   */
  public static boolean pointSeeable(
      Pose2d startPoint, Pose2d setPoint, List<Obstacle> obstaclelist) {
    AtomicBoolean seeable = new AtomicBoolean(true);
    Line2D beeline =
        new Line2D.Double(startPoint.getX(), startPoint.getY(), setPoint.getX(), setPoint.getY());
    obstaclelist.forEach(
        obstacle ->
            seeable.set(
                seeable.get() && shapeIntersectsLine(obstacle.obstacleProjection(), beeline)));
    return seeable.get();
  }

  public static boolean shapeIntersectsLine(java.awt.Shape shape, Line2D line) {
    PathIterator iterator = shape.getPathIterator(nullTransform);
    double[] currCoords = new double[6];
    double[] prevCoords = new double[2];

    iterator.currentSegment(prevCoords);

    while (!iterator.isDone()) {
      int segmentType = iterator.currentSegment(currCoords);

      if (segmentType != PathIterator.SEG_MOVETO) {
        Line2D.Double segment =
            new Line2D.Double(prevCoords[0], prevCoords[1], currCoords[0], currCoords[1]);

        if (segment.intersectsLine(line)) {
          return true;
        }
      }

      prevCoords[0] = currCoords[0];
      prevCoords[0] = currCoords[1];

      iterator.next();
    }
    return false;
  }

  /**
   * Nonstatic method to find out whether the goal of the robot can be driven straight to from where
   * the robot is.
   *
   * @return whether the goal can be driven straight to.
   */
  public boolean setPointSeeable() {
    return pointSeeable(startingPoint, endingPoint, obstacles);
  }
}
