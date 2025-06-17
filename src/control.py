import threading
import time

import roboticstoolbox as rtb

from ComauRobot import ComauRobot
from CoppeliaSimAPI import CoppeliaSimAPI
from CoppeliaSimRobot import CoppeliaSimRobot


def simulation_thread(sim, lock):
    while True:
        with lock:
            # print("stepping")
            # print(f"Simulation time: {sim.get_simulation_time():.2f} [s]")
            sim.step()
        time.sleep(0.01)  # tiny sleep to allow other threads to run


if __name__ == "__main__":

    sim = CoppeliaSimAPI(port=23000)
    coppelia_robot = CoppeliaSimRobot(sim)
    rtb_robot = ComauRobot()

    time_to_run = 5  # seconds
    num_steps = int(time_to_run / sim.dt)
    print("Number of steps to run:", num_steps)
    qt = rtb.jtraj(rtb_robot.qz, rtb_robot.qr, int(num_steps / 2))
    print(rtb_robot)
    sim.start()

    time.sleep(2)

    lock = threading.RLock()

    # Start simulation stepping in a separate thread
    print("Starting simulation thread...")
    sim_thread = threading.Thread(target=simulation_thread, args=(sim, lock))
    sim_thread.start()
    print("Got past simulation thread start.")

    # Send several joint positions in sequence
    # for q in qt.q:
    #     print("Setting joint target position:", q)
    #     with lock:
    #         coppelia_robot.setJointTargetPosition(q)

    # Send to a single joint position
    with lock:
        coppelia_robot.setJointTargetPosition(rtb_robot.qr)

    sim_thread.join()  # Wait for simulation to finish

    print("Simulation completed.")
    input("Press Enter to continue...")
    sim.stop()
