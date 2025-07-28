# Isaac Lab Tutorial Project

## Overview

This project/repository serves as a template for building projects or extensions based on Isaac Lab.
It allows you to develop in an isolated environment, outside of the core Isaac Lab repository.

**Key Features:**

- `Isolation` Work outside the core Isaac Lab repository, ensuring that your development efforts remain self-contained.
- `Flexibility` This template is set up to allow your code to be run as an extension in Omniverse.

**Keywords:** extension, template, isaaclab

## Installation

**Install Isaac Lab**

```bash
$docker login nvcr.io
$Username: $oauthtoken
$password: (enter your token fro nvidia webside)
```


🛑 **READ HERE:** To create a new project without cloning this repository and starting from initial IsaacLabTutorial see

 [➡ New Setup From Scratch](#new-setup-from-scratch) 

 This is the modofoed repository while I was following the tutorial and playing with the code

🛑 **READ HERE:** To work with WebRTC streaming (without GUI) from the local computer see the usefull commands: 

[➡ Usefull WebRTC Commands](#useful-webrtc-commands)


Clone isaaclab repository
```bash
$cd workspaces/
git clone git@github.com:isaac-sim/IsaacLab.git
```
-> [IsaacLab GitHub repository](https://github.com/isaac-sim/IsaacLab)

When you run ./docker/container.py start, it builds the image if it hasn't already been built, then runs it as a Docker container with mounted folders and optional extensions (like ROS 2, WebRTC, etc.).

```bash
$cd IsaacLab
$./docker/container.py start base
```

The information related to using IsaacLab in container:
https://isaac-sim.github.io/IsaacLab/main/source/deployment/docker.html


**Clone this project/repository separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory but in same folder)**

To run the container with both IsaacLab and our repository mounted enter:

```bash
$ cd IsaacLab/docker
$ ./container.py start base --files ~/workspaces/isaac_lab_tutorial/isaac_lab_tutorial.yaml
```

Inside container run the setup_isaac_lab.sh file to source isaac_lab_tutorial package and to use python interpreter that  Isaac Lab environment installed. (be sure $chmod +x setup_isaac_lab.sh)

```bash
$ cd workspace/IsaacLabTutorial
$ source ./setup_isaac_lab.sh
```

Be sure make "source" in order to keep the enviromental variables after .sh script finished.

**Verify that the extension is correctly installed by listing the available tasks**

```bash
$ cd workspace/IsaacLabTutorial
$ python scripts/list_envs.py
```

**Running a task:**

```bash
# use 'FULL_PATH_TO_isaaclab.sh|bat -p' instead of 'python' if Isaac Lab is not installed in Python venv or conda
$ python scripts/skrl/train.py --task=Template-Isaac-Lab-Tutorial-Direct-v0
```


## New Setup From Scratch

Enter isaac-lab-base container and run
```bash
$ cd isaaclab
$ ./isaaclab.sh --new
```
Be sure to select **External** and **Direct** | single agent. For the frameworks, select **skrl** and both **PPO** and **AMP** on the following menu. You can select other frameworks if you like, but this tutorial will detail skrl specifically. The configuration process for other frameworks is similar. You can get a copy of this code directly by checking out the initial branch of the tutorial repository!

This will create an extension project with the specified name at the chosen path. For me, I chose the name isaac_lab_tutorial.

In the container run following commands to see everthing works perfectly or not
Enter isaac-lab-base container and run
```bash
$ cd isaaclab
$ python -m pip install -e source/isaac_lab_tutorial
$ python scripts/list_envs.py
$ python scripts/skrl/train.py --task=Template-Isaac-Lab-Tutorial-Direct-v0
```
To export newly created package inside container run the following command (outside the docker)

```bash
$ docker cp isaac-lab-base:/workspace/isaac_lab_tutorial ~/workspaces/
```

I added isaac_lab_tutorial.yaml file to mount the exported package to the isaaclab container. So in the following command --file combines the docker-compose.yalm file of the IsaacLab and ours "isaac_lab_tutorial.yaml" With this way the  default IsaacLab docker building procedure will be kept all dependencies are well set and uploaded and our package can work with it.

```bash
$ cd IsaacLab/docker
$ ./container.py start base --files ~/workspaces/isaac_lab_tutorial/isaac_lab_tutorial.yaml
```

## Usefull WebRTC Commands

The command below retrieves the local IP address of your machine and launches the simulation script (launch_app.py) with livestreaming and camera features enabled. It sets environment variables (LIVESTREAM=2, ENABLE_CAMERAS=1, and PUBLIC_IP) and passes the local IP and a specific port (49100) to the Isaac Sim livestream configuration via --kit_args. If you want more detailed information about the argumants see the spesific directory <span style="color:red">/IsaacLab/source/isaaclab/isaaclab/app/app_launcher.py</span>



This allows the simulation to be accessed remotely for visualization through the specified IP and port. 


```bash
LOCAL_IP=$(hostname -I | awk '{print $1}') && LIVESTREAM=2 ENABLE_CAMERAS=1 PUBLIC_IP=$LOCAL_IP python scripts/tutorials/00_sim/launch_app.py --size 0.3 --kit_args "--/app/livestream/publicEndpointAddress=$LOCAL_IP --/app/livestream/port=49100"
```

<span style="color:gray">I added LOCAL_IP, --kit_args, PUBLIC_IP, etc., to the **setup_isaac_lab.sh** script. Now, if the computer configuration does not change, running this script once is equivalent to setting those parameters manually on the command line. You can customize **setup_isaac_lab.sh** according to your needs </span>



```bash
$ cd isaac_lab_tutorial
python scripts/tutorials/00_sim/launch_app.py --size 0.3
```


### Setup as Omniverse Extension (Optional)

We provide an example UI extension that will load upon enabling your extension defined in `source/isaac_lab_tutorial/isaac_lab_tutorial/ui_extension_example.py`.

To enable your extension, follow these steps:

1. **Add the search path of this project/repository** to the extension manager:
    - Navigate to the extension manager using `Window` -> `Extensions`.
    - Click on the **Hamburger Icon**, then go to `Settings`.
    - In the `Extension Search Paths`, enter the absolute path to the `source` directory of this project/repository.
    - If not already present, in the `Extension Search Paths`, enter the path that leads to Isaac Lab's extension directory directory (`IsaacLab/source`)
    - Click on the **Hamburger Icon**, then click `Refresh`.

2. **Search and enable your extension**:
    - Find your extension under the `Third Party` category.
    - Toggle it to enable your extension.

