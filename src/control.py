import sys
import threading
import time

from ComauRobot import ComauRobotDH  # ComauRobot, ComauRobotURDF
from CoppeliaSimAPI import CoppeliaSimAPI
from CoppeliaSimRobot import CoppeliaSimRobot
from KinematicControl import RegulationControl, TrajectoryControl  # KinematicControl
from Task import Task

lock = threading.RLock()


def simulation_thread(sim, lock):
    while True:
        with lock:
            sim.step()
        time.sleep(0.001)  # tiny sleep to allow other threads to run


def run_task(q_seq, coppelia_robot, lock, sim):
    for q in q_seq:
        # print("Setting joint target position:", q)
        with lock:
            coppelia_robot.setJointTargetPosition(q)  # Set the joint target position in CoppeliaSim
        time.sleep(sim.dt)


def wait_for_keypress():
    """Wait for the user to press Enter, without blocking other threads."""
    event = threading.Event()

    def input_thread():
        input("Press Enter to continue...")
        event.set()

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()
    while not event.is_set():
        time.sleep(0.05)  # Let other threads run


if __name__ == "__main__":

    port = 23000
    sim = None
    try:
        if len(sys.argv) > 1:
            port = int(sys.argv[1])

        sim = CoppeliaSimAPI(port=port)
        coppelia_robot = CoppeliaSimRobot(sim)
        # robot = ComauRobotURDF()
        robot = ComauRobotDH()

        traj_control = TrajectoryControl(robot)
        reg_control = RegulationControl(robot)
        task = Task(sim, robot, traj_control, reg_control)

        print(robot)
        sim.start()

        # Start simulation stepping in a separate thread
        print("Starting simulation thread...")
        sim_thread = threading.Thread(target=simulation_thread, args=(sim, lock))
        sim_thread.start()
        print("Got past simulation thread start.")

        print("Robot joint configuration: {}".format(robot.q))
        print("Robot pose: {}".format(robot.fkine(robot.q)))

        # Going to start position
        with lock:
            print("Setting robot to ready configuration...")
            coppelia_robot.setJointTargetPosition(robot.qr)
            robot.q = robot.qr  # Update robot configuration

        time.sleep(3)
        print("Robot joint configuration: {}".format(robot.q))
        print("Robot pose: {}".format(robot.fkine(robot.q)))
        wait_for_keypress()

        run_full_task = False
        if run_full_task:
            # Run full task
            res = task.run()
            print("Finished task control.")
            q_seq = res.control_result.q_list
            print("Task control completed. Number of joint positions:", len(q_seq))
        else:  # Run step by step
            # Run task 0
            print("Starting task #0.")
            res = task.t0_ready_position()
            print("Number of points in task #0:", len(res.control_result.q_list))
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #0:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #0.")
            wait_for_keypress()

            # Run task 1
            print("Starting task #1.")
            res = task.t1_qr_to_P0()
            print("Number of points in task #1:", len(res.control_result.q_list))
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #1:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #1.")
            wait_for_keypress()

            # Run task 2
            print("Starting task #2.")
            res = task.t2_P0_to_P1()
            print("Number of points in task #2:", len(res.control_result.q_list))
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #2:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #2.")
            wait_for_keypress()

            print("Skipping task #3.")
            print("Skipping task #3.")
            print("Skipping task #3.")

            print("Starting task #4.")
            res = task.t4_P1_to_P0()
            print("Number of points in task #4:", len(res.control_result.q_list))
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #4:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #4.")
            wait_for_keypress()

            print("Starting task #5.")
            res = task.t5_P0_to_P5()
            print("Number of points in task #5:", len(res.control_result.q_list))
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #5:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #5.")
            wait_for_keypress()

            print("Skipping task #6.")
            print("Skipping task #6.")
            print("Skipping task #6.")

            print("Starting task #7.")
            res = task.t7_P5_to_P0()
            print("Number of points in task #7:", len(res.control_result.q_list))
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #7:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #7.")
            wait_for_keypress()

            print("Starting task #8.")
            res = task.t8_P0_to_P6()
            run_task(res.control_result.q_list, coppelia_robot, lock, sim)
            print("Number of points in task #8:", len(res.control_result.q_list))
            print("Final pose =\n {}".format(robot.fkine(res.control_result.q_list[-1])))
            print("Target pose =\n {}".format(res.target_pose))
            print(
                "Final error =\n {}".format([round(e, 3) for e in res.control_result.err_list[-1]])
            )
            print("Finished task #8.")
            wait_for_keypress()

            print("Skipping task #9.")
            print("Skipping task #9.")
            print("Skipping task #9.")

        print("Simulation completed.")
        wait_for_keypress()
        with lock:
            print("Stopping simulation...")
            sim.sim.setStepping(False)
            sim.stop()
            sim.client.close()

        sim_thread.join()  # Wait for simulation to finish

        exit()

    except Exception as e:
        print("An error occurred:", e)
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
