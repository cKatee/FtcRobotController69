package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.geometry.path;

public class pathFollower {

    // the controller we will use to move the robot to each point in the path
    protected MotionProfiledPoseStablization controller;
    // the path of positions we want to traverse
    protected path targetPath;

    // index of the path we are on.
    protected int pathFollowingIndex = 0;

    // state of the path follower
    private pathFollowingStates path_following_state = pathFollowingStates.START;


    // when true, allows path to be followed again and again until the method is not being called.
    public boolean allowPathRerun = false;

    public pathFollower() {
    }


    public pathFollower(MotionProfiledPoseStablization controller) {
        this.controller = controller;
    }

    public pathFollower(MotionProfiledPoseStablization controller, path path) {
        this.targetPath = path;
    }

    public void setTargetPath(path targetPath) {
        this.targetPath = targetPath;
    }

    public void setController(MotionProfiledPoseStablization controller) {
        this.controller = controller;
    }

    /**
     * traverses the path
     * @return true if the path is completed, false if still following.
     */
    public boolean followPath() {

        // checks if the path has been completed and if it should replay the path

        if (targetPath.pathHasBeenCompleted) {
            controller.robot.drive.STOP();
            return true;
        }

        switch (path_following_state) {
            case START:
                path_following_state = pathFollowingStates.GOING_TO_POINT;
                break;
            case GOING_TO_POINT:
                // go to position and check if we are within the tolerance
                if (controller.goToPosition(targetPath.getPathPosition(pathFollowingIndex),targetPath.getPathPosition(pathFollowingIndex).getSteady_state_tolerance())) {

                    // if we are not on the last point than advance to the next one, if we are on the last point than we are finisihed with the path
                    if (pathFollowingIndex < targetPath.pathSize() - 1) {
                        pathFollowingIndex += 1;
                    } else {
                        path_following_state = pathFollowingStates.FINISHED;
                    }

                }

                break;
            case FINISHED:
                // stop drive train and set the complete flag to true
                controller.robot.drive.STOP();
                targetPath.pathHasBeenCompleted = true;
                cleanUp();
                break;
        }

        return false;
    }

    /**
     * to be run after followPath is completed automatically by the subsystem.  This allows the class to be reused properly
     */
    protected void cleanUp() {
        path_following_state = pathFollowingStates.START;
        pathFollowingIndex = 0;
    }


    /**
     * get the current state of the path follower
     * @return state of the path follower
     */
    public pathFollowingStates getPath_following_state() {
        return path_following_state;
    }
}


