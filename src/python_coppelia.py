# # import sim  # CoppeliaSim’s remoteApi.py renamed to sim.py since V4.4
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient as sim
# import time, sys

# sim.simxFinish(-1)                      # close all (defensive)
# cid = sim.simxStart('127.0.0.1', 19997,
#                     True, True, 5000, 5)  # blocking connection
# if cid == -1:
#     sys.exit("Cannot connect")

# # synchronous mode keeps sim steps under our control
# sim.simxSynchronous(cid, True)
# sim.simxStartSimulation(cid, sim.simx_opmode_blocking)

# err, jh = sim.simxGetObjectHandle(
#         cid, 'UR5_joint1', sim.simx_opmode_blocking)
# for step in range(300):
#     sim.simxSetJointTargetVelocity(
#         cid, jh, 0.5, sim.simx_opmode_oneshot)
#     sim.simxSynchronousTrigger(cid)
#     time.sleep(0.01)

# sim.simxStopSimulation(cid, sim.simx_opmode_blocking)
# sim.simxFinish(cid)

import sys
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient("localhost", 23000)
sim = client.getObject("sim")
print("API version:", sim.getInt32Param(sim.intparam_program_version))

print("Trying to get Smartsix handle...")
smartsix = sim.getObject("/Smartsix")

if smartsix == -1:
    print("Failed to get Smartsix handle.")
    sim.stopSimulation()
    sys.exit("Exiting due to invalid Smartsix handle.")

print("Got Smartsix Robot !")


print("hello")


time.sleep(3)

start_time = time.time()
sim.startSimulation()


while time.time() - start_time < 10:
    print(f"Simulation time: {sim.getSimulationTime():.2f} [s]")
    sim.step()
    time.sleep(0.1)

sim.stopSimulation()
