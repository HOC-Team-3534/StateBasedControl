package frc;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/**
 * The core of being able to follow a {@link PathPlannerTrajectory} from a .path file from PathPlanner. The path is
 * generated upon initialization using the {@link #loadPath(String, double, double) loadPath(String pathName, double maxSpeed, double maxAccel)} method.
 * Use the {@link #getInitialState()} in order to see the initial {@link PathPlannerState}
 * Make sure to {@link #resetStart()} when ready to follow the path, then {@link #getCurrentState()}
 */
public class PathPlannerFollower {

    private final String PATH_FILE_NAME;
    private PathPlannerTrajectory path;
    private long START_TIME;

    public PathPlannerFollower(String pathName, double autonMaxSpeed, double autonMaxAccel) {
        PATH_FILE_NAME = pathName;
        long load_start = System.currentTimeMillis();
        loadPath(PATH_FILE_NAME, autonMaxSpeed, autonMaxAccel);
        System.out.println(String.format("Path: [%s] took %d milliseconds.", pathName, System.currentTimeMillis() - load_start));
    }

    private void loadPath(String pathName, double maxSpeed, double maxAccel) {
        this.path = PathPlanner.loadPath(pathName, maxSpeed, maxAccel);
    }

    public PathPlannerTrajectory.PathPlannerState getState(double seconds) {
        return (PathPlannerTrajectory.PathPlannerState) path.sample(seconds);
    }

    public PathPlannerTrajectory.PathPlannerState getCurrentState() {
        double timeSinceStart = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return (PathPlannerTrajectory.PathPlannerState) path.sample(timeSinceStart);
    }

    public PathPlannerState getInitialState() {
        return path.getInitialState();
    }

    public double getRemainingTimeSeconds() {
        double timeSinceStart = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return path.getTotalTimeSeconds() - timeSinceStart;
    }

    public void resetStart() {
        START_TIME = System.currentTimeMillis();
    }

    public boolean isFinished() {
        double timeSinceStart = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return timeSinceStart >= path.getTotalTimeSeconds();
    }
}
