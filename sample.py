import roboticstoolbox as rtb
import spatialmath as sm
import spatialmath.base as smb
import numpy as np
from swift import Swift, Slider, Button
from Sawyer_model.sawyer import Sawyer

import ipdb
import copy
import time

# Make and instance of the Swift simulator and open it
env = Swift()
env.launch(realtime=True)
env.set_camera_pose([0, 0, 2], [0, 0, 0])

robot = Sawyer(env)
robot.q = robot._neutral
robot.add_to_env(env)


# Loop through each link in the Panda and if it is a variable joint,
# add a slider to Swift to control it
j = 0
for link in robot.links:
    if link.isjoint:
        # We use a lambda as the callback function from Swift
        # j=j is used to set the value of j rather than the variable j
        # We use the HTML unicode format for the degree sign in the unit arg
        env.add(
            Slider(
                lambda x, j=j: robot.set_joint_value(j, x),
                min=np.round(np.rad2deg(link.qlim[0]), 2),
                max=np.round(np.rad2deg(link.qlim[1]), 2),
                step=1,
                value=np.round(np.rad2deg(robot.q[j]), 2),
                desc="Sawyer Joint " + str(j),
                unit="&#176;",
            )
        )

        j += 1


env.add(
    Button(
        lambda x: env.close(),
        desc="Close",
    )
)

env.add(
    Button(
        lambda x: robot.move(),
        desc="Confirm",
    )
)

env.add(
    Button(
        lambda x: robot.home(),
        desc="Home",
    )
)

env.add(
    Button(
        lambda x: robot.ee_plus_z(),
        desc="+z",
    )
)

env.add(
    Button(
        lambda x: robot.ee_minus_z(),
        desc="-z",
    )
)

env.add(
    Button(
        lambda x: robot.ee_plus_x(),
        desc="+x",
    )
)

env.add(
    Button(
        lambda x: robot.ee_minus_x(),
        desc="-x",
    )
)

env.add(
    Button(
        lambda x: robot.ee_plus_y(),
        desc="+y",
    )
)

env.add(
    Button(
        lambda x: robot.ee_minus_y(),
        desc="-y",
    )
)

env.add(
    Button(
        lambda x: robot.ee_plus_z_ori(),
        desc="+Rz",
    )
)

env.add(
    Button(
        lambda x: robot.ee_minus_z_ori(),
        desc="-Rz",
    )
)

env.add(
    Button(
        lambda x: robot.ee_plus_y_ori(),
        desc="+Ry",
    )
)

env.add(
    Button(
        lambda x: robot.ee_minus_y_ori(),
        desc="-Ry",
    )
)

env.add(
    Button(
        lambda x: robot.ee_plus_x_ori(),
        desc="+Rx",
    )
)

env.add(
    Button(
        lambda x: robot.ee_minus_x_ori(),
        desc="-Rx",
    )
)



while True:
    # Process the event queue from Swift, this invokes the callback functions
    # from the sliders if the slider value was changed
    # env.process_events()

    # Update the environment with the new robot pose
    env.step(0)
    time.sleep(0.01)


