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
    private final GridBox[][] baseField = new GridBox[(int) INT_FIELD_LENGTH / GRID_SIDE_LENGTH][(int) INT_FIELD_WIDTH / GRID_SIDE_LENGTH];
    
    private GridBox[][] fullField = baseField;

    /**
     * Constructor. Instantiates a Gridded Field.
     * @param stationaryObstacles A list of obstacles which contains all the permanent, 
     * stationary 2-dimensional obstacles on the field.
     */
    public GriddedField(List<Obstacle> stationaryObstacles) {
        this.stationaryObstacles = stationaryObstacles;

        addObstacles(stationaryObstacles, baseField);
    }

    /**
     * Adds temporary obstacles to the field. This particular method removes all temporary obstacles 
     * currently on the field, then adds the ones inputted onto the field.
     * @param obstacles A list of Obstacles, representing the temporary obstacles being placed on the field.
     * These temporary obstacles are best for moving objects, where their position can be updated by clearing
     * the field and re-placing them in their new position every period.
     */
    public void addTempObstacles(List<Obstacle> obstacles) {
        resetTemps();
        addObstacles(obstacles, fullField);
    }

    /**
     * A static method which adds temporary obstacles to a field. This method does not remove the current 
     * temporary obstacles on the field.
     * @param obstacles A list of Obstacles, representing the temporary obstacles being placed on the field.
     * These temporary obstacles are best for moving objects, where their position can be updated by clearing
     * the field and re-placing them in their new position every period.
     * @param field A GridBox two-dimensional array which represents the field to which the obstacles will be added into.
     */
    public static void addObstacles(List<Obstacle> obstacles, GridBox[][] field) {
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
                    if (obstacle.contains(x,y)) {
                        field[x][y].obstaclize();
                    }
                }
            }     
        }
    }


    /** Removes the temporary obstacles from the field. */
    public void resetTemps() {
        fullField = baseField;
    }

    /** @return The GridBox double array representing the field. */
    public GridBox[][] field() {
        return fullField;
    }

}
