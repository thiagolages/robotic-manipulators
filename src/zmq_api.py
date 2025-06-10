from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# port as set via -GzmqRemoteApi.rpcPort
client = RemoteAPIClient("localhost", 23001)
sim = client.require("sim")

sim.setStepping(True)

sim.startSimulation()
while (t := sim.getSimulationTime()) < 3:
    print(f"Simulation time: {t:.2f} [s]")
    sim.step()

print("API version:", sim.getInt32Param(sim.intparam_program_version))
sim.stopSimulation()
