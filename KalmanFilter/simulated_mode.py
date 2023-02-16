import time
import pythonVersion as setup
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


def run_simulated():
    plt.style.use('seaborn-pastel')

    fig, ax = plt.subplots(1, 3)
    fig.suptitle("Kalman Filter")
    ax[0].set_box_aspect(1)
    ax[0].set_xbound(-15,15)
    ax[0].set_ybound(-15,15)
    ax[0].set_xlim(-15,15)
    ax[0].set_ylim(-15,15)
    ax[0].set_axis_on()
    ax[0].set_xlabel("Acceleration")

    ax[1].set_box_aspect(1)
    ax[1].set_xlim(-15, 15)
    ax[1].set_ylim(-15, 15)
    ax[1].set_xbound(-15,15)
    ax[1].set_ybound(-15,15)
    ax[1].set_axis_on()
    ax[1].set_xlabel("Velocity")

    ax[2].set_box_aspect(1)
    ax[2].set_xlim(-15, 15)
    ax[2].set_ylim(-15, 15)
    ax[2].set_xbound(-15,15)
    ax[2].set_ybound(-15,15)
    ax[2].set_axis_on()
    ax[2].set_xlabel("Position")

    
    sim = setup.Simulation()

    def anim(i):
        sim.step()
        drone = sim.drone
        
        return (ax[0].plot(drone.acceleration,"bo") + ax[0].plot(sim.filter.state[0],"rx")
            + ax[1].plot(drone.velocity, "bo") + ax[1].plot(sim.filter.state[1], "rx")
            + ax[2].plot(drone.position, "bo") + ax[2].plot(sim.filter.state[2], "rx"))


    anim = FuncAnimation(fig, anim, frames=200, interval=20, blit=True)

    plt.show()

    # for i in range(200):
    #     sim.step()
    #     drone = sim.drone
    #     print(drone.acceleration)
    #     print(drone.velocity)
    #     print(drone.position)
    #     time.sleep(0.1)

def main():
    run_simulated()


if __name__ == '__main__':
    main()