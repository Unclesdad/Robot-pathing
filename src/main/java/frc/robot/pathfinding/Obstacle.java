package frc.robot.pathfinding;

import java.awt.geom.*;

import frc.robot.Constants;

public class Obstacle {
    public java.awt.Shape obstacle;
    public Rectangle2D noRobotBoundingBox = 
        obstacle.getBounds2D();

    public Area obstacleTrueArea = new Area(obstacle)
        .createTransformedArea(
            AffineTransform
                .getScaleInstance(
                    (1 + Constants.ROBOT_RADIUS * 2 / noRobotBoundingBox.getHeight()), 
                    (1 + Constants.ROBOT_RADIUS * 2 / noRobotBoundingBox.getWidth())));
    
    public Rectangle2D boundingBox = 
        obstacleTrueArea.getBounds2D();

    public Obstacle(java.awt.Shape obstacle) {
        this.obstacle = obstacle;
    }

    public Rectangle2D boundingBox() {
        return boundingBox;
    }
}
