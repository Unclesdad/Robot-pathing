package frc.robot.pathfinding;

import java.awt.Shape;
import java.awt.geom.Rectangle2D;
import java.util.List;
import java.util.ListIterator;

import frc.robot.Constants.Field;

public class GriddedField {
    public static final int GRID_SIDE_LENGTH = 4;
    
    public static final int INT_FIELD_LENGTH = (int) Field.FIELD_LENGTH;
    public static final int INT_FIELD_WIDTH = (int) Field.FIELD_WIDTH;

    public final List<Obstacle> stationaryObstacles;

    /* Increasing the grid size to make it GRID_SIDE_LENGTH on each side, because otherwise 
    * it would take up too much processing power and we don't need that level of precision. */
    boolean[][] baseField = new boolean[(int) INT_FIELD_LENGTH / GRID_SIDE_LENGTH][(int) INT_FIELD_WIDTH / GRID_SIDE_LENGTH];
    
    boolean[][] fullField = baseField;

    /**
     * Constructor. Instantiates a Gridded Field.
     * @param stationaryObstacles A list of obstacles which contains all the permanent, 
     * stationary 2-dimensional obstacles on the field
     */
    public GriddedField(List<Obstacle> stationaryObstacles) {
        this.stationaryObstacles = stationaryObstacles;

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
        }
    }

    /**
     * Adds temporary obstacles to the field. This particular method removes all temporary obstacles 
     * currently on the field, then adds the ones inputted onto the field.
     * @param obstacles A list of Obstacles, representing the temporary obstacles being placed on the field.
     * These temporary obstacles are best for moving objects, where their position can be updated by clearing
     * the field and re-placing them in their new position every period.
     */
    public void addTempObstacles(List<Obstacle> obstacles) {
        addTempObstacles(obstacles, true);
    }

    /**
     * Adds temporary obstacles to the field. 
     * @param obstacles A list of Obstacles, representing the temporary obstacles being placed on the field.
     * These temporary obstacles are best for moving objects, where their position can be updated by clearing
     * the field and re-placing them in their new position every period.
     * @param resetTemps A boolean which determines whether the field should be cleared of its current temporary
     * obstacles. If true -> temporary obstacles are cleared. If false -> temporary obstacles are not cleared and
     * the inputted obstacles are place on top of the current ones.
     */
    public void addTempObstacles(List<Obstacle> obstacles, boolean resetTemps) {
        if (resetTemps) {
            fullField = baseField;
        }
        ListIterator iterator = obstacles.listIterator();

        while (iterator.hasNext()) {
            Shape obstacle = obstacles.get(iterator.nextIndex()).projectedObstacle;
            Rectangle2D obstacleBoundingBox = obstacles.get(iterator.nextIndex()).projectedObstacle.getBounds2D();
            int boundingX = (int) obstacleBoundingBox.getX();
            int boundingY = (int) obstacleBoundingBox.getY();
            int boundingWidth = (int) obstacleBoundingBox.getWidth();
            int boundingHeight = (int) obstacleBoundingBox.getHeight();
    
            for (int x = boundingX; x <= boundingX + boundingWidth; x++) {
                for (int y = boundingY; y <= boundingY + boundingHeight; y++) {
                    fullField[x][y] = obstacle.contains(x,y);
                }
            }     
        }
    }

    public void removeTempObstacles() {
        fullField = baseField;
    }

    public boolean[][] field() {
        return fullField;
    }

}
