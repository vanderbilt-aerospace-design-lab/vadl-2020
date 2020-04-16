# VADL 2020

## Introduction

This repository contains the software developed by the Vanderbilt Aerospace Design Laboratory (VADL) 2020 design team. Each year the team competes in the [NASA University Student Launch Competition](https://www.nasa.gov/stem/studentlaunch/home/index.html). The NASA-defined goal of the 2019-2020 competition was to launch a rocket and payload that can recover a lunar ice sample from an unspecified location. VADL chose to design an unmanned aerial vehicle (UAV) that would be stored inside the rocket, deploy from the rocket after the rocket has landed on the ground, and autonomously complete a lunar sample collection mission. In addition, the team decided to tackle three extra challenges:

**Air-Based Sampling:** The UAV will collect the sample by hovering over the sampling zone and lowering a sampling tool to the ground. Lunar sample collection can be very challenging due to the diversity of environments that can be found on the moon. Since the UAV does not have to land to sample, it will be able to sample from areas inaccessible to a rover or traditional UAV, such as steep slopes and narrow or rocky terrain.

**Fully Autonomous Flight:** The UAV will be capable of autonomous image-based guidance, navigation, and control. It will be able to operate independently from GPS and magnetometers. Autonomous flight is essential so that the UAV can operate in real-time, as opposed to being teleoperated from Earth.

**Contact Charging Station:** A charging station has been added to the rocket as an alternative to carrying a large power source onboard the UAV. This will allow the UAV to be much lighter, with improved control and flight time, and enable repeatable missions after recharging.

The VADL 2020 team's goal is to create a novel UAV and rocket that solve some of the greatest challenges facing space exploration: robust sample collection, real-time navigation and control, and repeatibility. 

The picture below shows our fullscale rocket and payload. The rocket is ~10ft long and 5.5" diameter. The UAV and charging station are housed in the payload bay. Once the rocket is launched and reaches apogee, drogue and main parachutes are used to safely recover the rocket on the ground. Once on the ground, the rocket autonomously reorients and opens the payload bay (which is split into thirds) to allow for the UAV and charging station to deploy.

![Fullscale rocket and UAV payload](/images/rocket.png)


The UAV has foldable arms and legs so it can fit in the rocket. Once the payload bay opens, the UAV arms and legs passively open and lock into place. After the sample collection mission, the UAV returns to the rocket and lands on the charging station. The UAV has metal contacts on two of its legs that allow for current to flow to the battery once they contact the metal charging platform.

![UAV folded/unfolded configurations and charging station](images/uav_triad.png)

The photos below show the UAV in action during a flight test. The right photo showcases air-based sampling. The sampling tool is lowered on a winch and is electrically independent from the main UAV.

![UAV air-based sampling and landing](images/payload_action_shot.png)


# Autonomy

VADL has developed software for UAV autonomous flight using both GPS and [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) localization. GPS is great because it's already available for most drones and doesn't require much work to implement. The downside is that it can only localize the drone within a few meters accuracy. SLAM is a method of localizing the UAV in an unknown environment without any external aids, such as GPS. 

ELABORATE MORE HERE. Maybe put the state machine and break down the mission sequence?

# Project Status

Due to the coronavirus pandemic, the in-person NASA competition was canceled. Fortunately, the team has already finished building the rocket and UAV and has successfully performed a fullscale launch and sample collection mission. GPs-based autonomy has been implemented, but SLAM autonomy is still in development. We have demonstrated extremely stable indoor SLAM flight and have flown outdoors with moderate amounts of our drift. Some of our results can be seen [here].

