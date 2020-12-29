package org.firstinspires.ftc.teamcode.geometry;


import java.util.ArrayList;

/**
 * path for following complex movements easily
 */
public class path {

    // false unless the path is completed.  This is to be set by the path following algorithm when the final point in the path has reach an acceptable steady state
    public boolean pathHasBeenCompleted = false;

    private ArrayList<pathPosition> pathPositions = new ArrayList<>();

    /**
     * blank constructor pog moment
     */
    public path() {

    }

    /**
     * contructor with path
     * @param pathPositions the arraylist of paths lol
     */
    public path(ArrayList<pathPosition> pathPositions) {
        this.pathPositions = pathPositions;
    }

    /**
     * appends a new pathPosition to the path
     * @param pos new pathPosition
     */
    public void addPosition(pathPosition pos) {
        pathPositions.add(pos);
    }

    /**
     * appends a new position to the path as a pathPosition
     * @param pos new position
     */
    public void addPosition(position pos) {
        pathPositions.add(new pathPosition(pos));
    }

    /**
     * gets the entire list of positions
     * @return pathPositions
     */
    public ArrayList<pathPosition> getAllPathPositions() {
        return pathPositions;
    }

    /**
     * get pathPosition at a given index
     * @param index the index we want starting from 0
     * @return
     */
    public pathPosition getPathPosition(int index) {
        return pathPositions.get(index);
    }


    /**
     * gets the first position of the path
      * @return the first pathPosition
     */
    public pathPosition getFirstPosition() {
        return getPathPosition(0);
    }

    /**
     * gets the last position of the path
     * @return the last pathPosition
     */
    public pathPosition getLastPosition() {
        return pathPositions.get(pathPositions.size() - 1);
    }

    public long pathSize() {
        return pathPositions.size();
    }
}
