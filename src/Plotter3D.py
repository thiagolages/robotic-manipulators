from threading import RLock

from CoppeliaSimAPI import CoppeliaSimAPI


class Plotter3D:
    def __init__(
        self,
        sim: CoppeliaSimAPI,
        lock: RLock,
        key: str = "customData.trajColorRGB",
        verbose: bool = False,
    ):

        # Sim object
        self.sim = sim.sim  # Access the sim object from CoppeliaSimAPI

        # Signal name, inside 3d_plot.lua, which is read every step
        self.key = key

        # Lock for thread safety
        self.lock = lock

        self.verbose = verbose

        # RGB colors
        self.no_color = "0,0,0"
        self.red_color = "1,0,0"
        self.green_color = "0,1,0"
        self.blue_color = "0,0,1"
        self.gray_color = "0.5,0.5,0.5"
        self.yellow_color = "1,1,0"

    def set_color(self, color=None):

        # Default to red if no color is provided
        color = self.red_color if color is None else color

        if isinstance(color, (tuple, list)):
            color = ",".join(map(str, color))  # Convert to string format 'r,g,b'

        color = color.split(",")  # Split the string into a list
        color = [float(c) for c in color]  # Ensure color is a list

        with self.lock:
            self.sim.setBufferProperty(
                self.sim.handle_scene, self.key, self.sim.packFloatTable(color)
            )

        if self.verbose:
            print("#############################################################")
            print("Changed colors in 3D plotter to: {}".format(color))
            print("#############################################################")

    def on(self):
        """
        Enable the the 3D plotter with red trajectory color.
        """
        self.set_color(self.red_color)

    def off(self):
        self.set_color(self.no_color)

    def red(self):
        self.set_color(self.red_color)

    def green(self):
        self.set_color(self.green_color)

    def blue(self):
        self.set_color(self.blue_color)

    def yellow(self):
        self.set_color(self.yellow_color)

    def gray(self):
        self.set_color(self.gray_color)


class MockPlotter:
    def __init__(self, verbose: bool = False):
        self.name = "Mock Plotter"
        self.verbose = verbose

    def on(self):
        if self.verbose:
            print("MockPlotter: Turning on")

    def off(self):
        if self.verbose:
            print("MockPlotter: Turning off")

    def red(self):
        if self.verbose:
            print("MockPlotter: Setting color to red")

    def green(self):
        if self.verbose:
            print("MockPlotter: Setting color to green")

    def blue(self):
        if self.verbose:
            print("MockPlotter: Setting color to blue")
