package frc.robot.pathfinding;

import java.awt.geom.*;

import frc.robot.Constants;

public class Obstacle {
    public java.awt.Shape obstacle;
    public Rectangle2D noRobotBoundingBox = 
        obstacle.getBounds2D();

    // the area in which the CENTER of the robot should not go, lest the EDGE of the robot will touch the EDGE of the obstacle.
    public Area projectedObstacle = new Area(obstacle)
        .createTransformedArea(
            AffineTransform
                /* scales the bound such that each side of the bound is extended out by the radius of the robot.
                 * This makes it so that the robot's position can be treated as a POINT which can't hit this
                 * "PROJECTED obstacle" rather than an area which can't hit this other area */
                .getScaleInstance(
                    (1 + Constants.ROBOT_RADIUS * 2 / noRobotBoundingBox.getHeight()), 
                    (1 + Constants.ROBOT_RADIUS * 2 / noRobotBoundingBox.getWidth())));
    
    // a precice rectangle which envelopes the whole of the obstacle.
    public Rectangle2D boundingBox = 
        projectedObstacle.getBounds2D();

    public Obstacle(java.awt.Shape obstacle) {
        this.obstacle = obstacle;
    }

    /** @return the bounding box for the obstacle. */
    public Rectangle2D boundingBox() {
        return boundingBox;
    }
}
