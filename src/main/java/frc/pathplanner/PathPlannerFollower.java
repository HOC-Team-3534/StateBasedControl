package frc.pathplanner;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.pathplanner.config.PathPlannerConfig;

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
        return (PathPlannerTrajectory.PathPlannerState) path.sample(timeSinceStart + PathPlannerConfig.LOOK_AHEAD_TIME);
    }

    /**
     * Shift the time being used to sample the trajectory to the closest point on
     * the path within a window of time, set in {@link PathPlannerConfig}
     * 
     * @param robotPose the current pose of the robot
     */
    public void shiftTimeToClosestPoint(Pose2d robotPose) {
        double currTime = getClosestStateTime(robotPose);
        START_TIME += (int) (currTime * 1000) - getTimeSinceStart();
    }

    int getIndexForTime(PathPlannerTrajectory traj, double time) {
        int low = 0;
        int high = traj.getStates().size() - 1;
        while (low != high) {
            int mid = (low + high) / 2;
            if (traj.getState(mid).timeSeconds < time) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        return low;
    }

    protected PathPlannerTrajectory getRelativelyCloseTrajectory(PathPlannerTrajectory traj,
                                                                 double time, double window) {
        double startTime = time - window / 2;
        double endTime = time + window / 2;
        int startIndex = getIndexForTime(traj, startTime);
        int endIndex = getIndexForTime(traj, endTime);
        List<State> states = traj.getStates().subList(startIndex, endIndex);
        return (PathPlannerTrajectory) new Trajectory(states);
    }

    protected double getClosestStateTime(Pose2d robotPose) {
        PathPlannerTrajectory traj = getRelativelyCloseTrajectory(this.path, getTimeSinceStart(), PathPlannerConfig.RESET_WINDOW_SIZE);
        int index = 0;
        double lowestDist = traj.getState(0).poseMeters.getTranslation().getDistance(robotPose.getTranslation());
        for (int i = 1; i < traj.getStates().size(); i += 2) {
            double dist = traj.getState(i).poseMeters.getTranslation().getDistance(robotPose.getTranslation());
            if (dist < lowestDist) {
                index = i;
                lowestDist = dist;
            }
        }
        return traj.getState(index).timeSeconds;
    }

    /**
     * @return the initial state of the trajectory
     */
    public PathPlannerState getInitialState() {
        return path.getInitialState();
    }

    private double getTimeSinceStart() {
        return (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
    }

    /**
     * @return the remaining time to completely follow the path
     */
    public double getRemainingTimeSeconds() {
        return path.getTotalTimeSeconds() - getTimeSinceStart();
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
