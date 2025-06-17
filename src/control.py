import sys
import threading
import time
import traceback

from ComauRobot import ComauRobotDH  # ComauRobot, ComauRobotURDF
from CoppeliaSimAPI import CoppeliaSimAPI
from CoppeliaSimRobot import CoppeliaSimRobot
from KinematicControl import RegulationControl, TrajectoryControl  # KinematicControl
from Plotter3D import Plotter3D
from Task import Task

lock = threading.RLock()


def simulation_thread(sim, lock):
    while True:
        with lock:
            sim.step()
        time.sleep(0.001)  # tiny sleep to allow other threads to run


def move_sim_robot(q_seq, coppelia_robot, lock, last_only=False):
    if last_only:
        with lock:
            coppelia_robot.setJointTargetPosition(q_seq[-1])  # Go to last position only
        time.sleep(1)  # Wait for the robot to reach the last position
        return

    for q in q_seq:
        with lock:
            coppelia_robot.setJointTargetPosition(q)  # Set the joint target position in CoppeliaSim
        time.sleep(0.01)  # This is the controller dt


def wait_for_keypress():
    """Wait for the user to press Enter, without blocking other threads."""
    event = threading.Event()

    def input_thread():
        input("Press Enter to continue...")
        event.set()

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()
    while not event.is_set():
        time.sleep(0.001)  # Let other threads run


if __name__ == "__main__":

    port = 23000
    sim = None
    verbose = False
    run_full_task = True  # Run full task at once or step by step
    try:
        if len(sys.argv) > 1:
            port = int(sys.argv[1])

        sim = CoppeliaSimAPI(port=port, verbose=0)
        coppelia_robot = CoppeliaSimRobot(sim)
        # robot = ComauRobotURDF()
        robot = ComauRobotDH()
        plotter = Plotter3D(sim, lock, verbose=verbose)
        plotter.off()  # Disable plotting initially

        traj_control = TrajectoryControl(robot, K=1.0, dt=sim.dt)
        reg_control = RegulationControl(robot, K=3.5, dt=sim.dt)
        task = Task(sim, robot, traj_control, reg_control, plotter, verbose=verbose)

        print(robot)

        # Start the simulation
        sim.start()

        # Start simulation stepping in a separate thread
        print("Starting simulation thread...")
        sim_thread = threading.Thread(target=simulation_thread, args=(sim, lock))
        sim_thread.start()

        if verbose:
            print("Robot joint configuration: {}".format(robot.q))
            print("Robot pose: {}".format(robot.fkine(robot.q)))

        # Going to start position
        with lock:
            print("Setting robot to ready configuration...")
            coppelia_robot.setJointTargetPosition(robot.qr)
            robot.q = robot.qr  # Update robot configuration

        time.sleep(5)  # Wait for the robot to reach the ready position
        # wait_for_keypress()

        msg = "Running full task..." if run_full_task else "Running task step by step..."
        print(msg)

        all_points = []
        for idx, task_step in enumerate(task.task_steps):
            if run_full_task:
                # Delay before starting square, diamond and circle tasks
                if idx in [3, 6, 9]:
                    time.sleep(1.75)  # Avoid plotting the flag before it's time

            print(f"Starting task #{idx}.")
            res = task_step()
            print("Number of points in task #{}: {}".format(idx, len(res.control_result.q_list)))
            if task.verbose:
                print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
                print("Target pose =\n {}".format(res.target_pose))
                print(
                    "Final error =\n {}".format(
                        [round(e, 3) for e in res.control_result.err_list[-1]]
                    )
                )
            q_seq = res.control_result.q_list
            all_points.extend(q_seq)
            move_sim_robot(q_seq, coppelia_robot, lock)
            print(f"Finished task #{idx}: {res.name}.")
            if not run_full_task:
                wait_for_keypress()

            task.plotter.off()

        print("Simulation completed.")
        print("Total points in trajectory:", len(all_points))
        wait_for_keypress()

        with lock:
            print("Stopping simulation...")
            sim.sim.setStepping(False)
            sim.stop()

        sim_thread.join()  # Wait for simulation to finish

        exit()

    except Exception as e:
        print("An error occurred:", e)
        traceback.print_exc()
        with lock:
            print("Stopping simulation...")
            sim.stop()
        sys.exit(1)
    finally:
        print("Simulation stepping stopped.")
        with lock:
            print("Stopping simulation...")
            sim.stop()
        print("Simulation stopped.")
