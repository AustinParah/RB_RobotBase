package frc.robot.async;

import edu.wpi.first.wpilibj.Timer;
//import frc.robot.subsystems.util.BSLogger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class AsyncManager {
    private final AsyncWorkThread thread = new AsyncWorkThread(0.005); // 5ms
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final List<AsyncTask> tasks = Collections.synchronizedList(new ArrayList<>());
    private double lastOverrunReport = 0;

    public AsyncManager() {
        thread.setDaemon(true);
        this.register("reportOverruns", this::reportOverruns);
    }

    public void start() {
        //BSLogger.log("AsyncManager", "Starting async tasks");
        running.set(true);
        thread.start();
    }

    public void stop() {
        //BSLogger.log("AsyncManager", "Stopping async tasks");
        running.set(false);
    }

    public void register(String taskName, Runnable runnable) {
        //BSLogger.log("AsyncManager", "Register: " + taskName);
        tasks.add(new AsyncTask(taskName, runnable));
    }

    public void unregister(String taskName) {
        //BSLogger.log("AsyncManager", "Unregister: " + taskName);
        tasks.removeIf(task -> task.getName().equals(taskName));
    }

    public int getNumberOfTasks() {
        return tasks.size();
    }

    void reportOverruns() {
        int numOverruns = thread.overRun.get();
        if ((numOverruns > 0) && (Timer.getFPGATimestamp() - lastOverrunReport > 5.0)) {
            //BSLogger.log("AsyncManager", "Overruns: " + numOverruns + " in the last 5 seconds (of " + thread.numRuns.get() + " runs);");
            lastOverrunReport = Timer.getFPGATimestamp();
            thread.overRun.set(0);
            thread.numRuns.set(0);
        }
    }

    class AsyncWorkThread extends Thread {
        private final double period;
        AtomicInteger overRun = new AtomicInteger(0);
        AtomicInteger numRuns = new AtomicInteger(0);
        private double lastRun = 0;

        AsyncWorkThread(double period) {
            this.period = period;
        }

        @Override
        public void run() {
            while (running.get()) {
                double current = Timer.getFPGATimestamp();
                // If the time since the last run is greater than the period, run the tasks
                if ((current - lastRun) > period) {
                    synchronized (tasks) {
                        for (AsyncTask task : tasks) {
                            task.run();
                        }
                    }
                    if ((Timer.getFPGATimestamp() - lastRun) > period * 2) {
                        overRun.incrementAndGet();
                    }
                    lastRun = Timer.getFPGATimestamp();
                    numRuns.incrementAndGet();
                }
            }
            //BSLogger.log("AsyncManager", "Existing async manager thread");
        }
    }
}
