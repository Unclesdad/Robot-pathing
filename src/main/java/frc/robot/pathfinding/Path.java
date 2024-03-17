package frc.robot.pathfinding;

import java.awt.geom.Line2D;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.math.geometry.Pose2d;

public class Path {
    public final Pose2d startingPoint;
    public final Pose2d endingPoint;

    public final List<Obstacle> obstacles;

    public Path(Pose2d startingPoint, Pose2d endingPoint, List<Obstacle> obstacles) {
        this.startingPoint = startingPoint;
        this.endingPoint = endingPoint;
        this.obstacles = obstacles;
    }

    public boolean setpointSeeable() {
        AtomicBoolean seeable = new AtomicBoolean(true);
        Line2D beeline = new Line2D.Double(
            startingPoint.getX(), 
            startingPoint.getY(), 
            endingPoint.getX(), 
            endingPoint.getY());
        obstacles.forEach(obstacle -> seeable.set(seeable.get() && beeline.intersects(obstacle.boundingBox())));
        return seeable.get();
    }
}
