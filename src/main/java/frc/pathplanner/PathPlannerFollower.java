package frc.pathplanner;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/**
 * The core of being able to follow a {@link PathPlannerTrajectory} from a .path
 * file from PathPlanner. The path is generated upon initialization using the
 * {@link #loadPath(String, double, double) loadPath(String pathName, double
 * maxSpeed, double maxAccel)} method. Use the {@link #getInitialState()} in
 * order to see the initial {@link PathPlannerState} Make sure to
 * {@link #resetStart()} when ready to follow the path, then
 * {@link #getCurrentState()}
 */
public class PathPlannerFollower {
    private final String PATH_FILE_NAME;
    private PathPlannerTrajectory path;
    private long START_TIME;

    /**
     * @param pathName      the literal name of the path file, without the extension
     * @param autonMaxSpeed the max speed of the robot while following the path
     *                      during autonomous
     * @param autonMaxAccel the max acceleration of the robot while following the
     *                      path during autonomous
     */
    public PathPlannerFollower(String pathName, double autonMaxSpeed, double autonMaxAccel) {
        PATH_FILE_NAME = pathName;
        long load_start = System.currentTimeMillis();
        loadPath(PATH_FILE_NAME, autonMaxSpeed, autonMaxAccel);
        System.out.println(String.format("Path: [%s] took %d milliseconds.", pathName, System.currentTimeMillis() - load_start));
    }

    /**
     * Load the path for the path follower from the file
     * 
     * @param pathName the literal name of the path file, without the extension
     * @param maxSpeed the max speed of the robot while following the path during
     *                 autonomous
     * @param mxAccel  the max acceleration of the robot while following the path
     *                 during autonomous
     */
    private void loadPath(String pathName, double maxSpeed, double maxAccel) {
        this.path = PathPlanner.loadPath(pathName, maxSpeed, maxAccel);
    }

    /**
     * Get the state the robot will at, at a particular time
     * 
     * @param seconds the time at which to determine the state of the robot given
     *                the path
     * 
     * @return the state the robot will be at, at the particular time
     */
    public PathPlannerTrajectory.PathPlannerState getState(double seconds) {
        return (PathPlannerTrajectory.PathPlannerState) path.sample(seconds);
    }

    /**
     * Given the time since the start of following the path, the state the robot
     * should currently be at
     * 
     * @return the current state the robot should be at
     */
    public PathPlannerTrajectory.PathPlannerState getCurrentState() {
        double timeSinceStart = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return (PathPlannerTrajectory.PathPlannerState) path.sample(timeSinceStart);
    }

    /**
     * @return the initial state of the trajectory
     */
    public PathPlannerState getInitialState() {
        return path.getInitialState();
    }

    /**
     * @return the remaining time to completely follow the path
     */
    public double getRemainingTimeSeconds() {
        double timeSinceStart = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return path.getTotalTimeSeconds() - timeSinceStart;
    }

    /**
     * reset the start time of the path to the current time. needed in order to
     * create path then follow it later, so the time must be rest.
     */
    public void resetStart() {
        START_TIME = System.currentTimeMillis();
    }

    /**
     * @return has the time since the start met or exceed the duration of time of
     *         the path
     */
    public boolean isFinished() {
        double timeSinceStart = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return timeSinceStart >= path.getTotalTimeSeconds();
    }
}
