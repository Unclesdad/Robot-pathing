package frc.robot.pathfinding;

import java.awt.Shape;
import java.awt.geom.Rectangle2D;
import java.lang.invoke.ConstantBootstraps;
import java.util.List;
import java.util.ListIterator;

import frc.robot.Constants;
import frc.robot.Constants.Field;

public class GriddedField {
    public static final int INT_FIELD_LENGTH = (int) Field.FIELD_LENGTH;
    public static final int INT_FIELD_WIDTH = (int) Field.FIELD_WIDTH;

    public final List<Obstacle> stationaryObstacles;
    public List<Obstacle> movingObstacles;

    /* Increasing the grid size to make it 2cm on each side, because otherwise it would 
    * take up too much processing power and we don't need that level of precision. */
    boolean[][] baseField = new boolean[INT_FIELD_LENGTH / 2][INT_FIELD_WIDTH / 2]; 

    public GriddedField(List<Obstacle> stationaryObstacles, List<Obstacle> movingObstacles) {
        this.stationaryObstacles = stationaryObstacles;
        this.movingObstacles = movingObstacles;

        ListIterator iterator = stationaryObstacles.listIterator();

        while (iterator.hasNext()) {
            Shape obstacle = stationaryObstacles.get(iterator.nextIndex()).projectedObstacle;
            Rectangle2D obstacleBoundingBox = stationaryObstacles.get(iterator.nextIndex()).projectedObstacle.getBounds2D();
            int boundingX = (int) obstacleBoundingBox.getX();
            int boundingY = (int) obstacleBoundingBox.getY();
            int boundingWidth = (int) obstacleBoundingBox.getWidth();
            int boundingHeight = (int) obstacleBoundingBox.getHeight();

            for (int x = boundingX; x <= boundingX + boundingWidth; x++) {
                for (int y = boundingY; y <= boundingY + boundingHeight; y++) {
                    baseField[x][y] = obstacle.contains(x,y);
                }
            }
            
            obstacleBoundingBox.getX();
        }

        
    }


}
