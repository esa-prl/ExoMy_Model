# ExoMy - Model Repository
*Disclaimer: This model and the simulation are still in development and subject to change. If you want to contribute to the project, contact us with your plans [Discord](https://discord.gg/gZk62gg).*

This repository contains a URDF representation of ExoMy, that can be used in the visualization tool rviz2 and the simulation environment Gazebo.

![rviz image](rviz.png)
![gazebo image](gazebo.png)


The software is supposed to run on a more powerful machine than the Raspberry Pi.
It was tested on a laptop with Ubuntu 20.04. 
## How to run
 
- Clone this repository `git clone https://github.com/esa-prl/ExoMy_Model` next to the `ExoMy_Software` folder
- Be sure to be on the branch `ros2/foxy` on both repositories
- To allow programs being opened from within the docker container run `xhost +local:docker`
- Change to the folder `ExoMy_Model/docker`
- Start the Docker container with `docker-compose run --name exomy_sim exomy_sim`
- Inside the docker run `colcon build`
- Followed by `source install/setup.zsh`
- Launch the simulation with `ros2 launch exomy_sim_launch simulation.launch.py`
- Rviz2 and gazebo should open and you can drive around with the gamepad


# ExoMy Links

### [Website](https://esa-prl.github.io/ExoMy/)

### [Wiki](https://github.com/esa-prl/ExoMy/wiki)

### [Documentation Repository](https://github.com/esa-prl/ExoMy)

### [Software Repository](https://github.com/esa-prl/ExoMy_Software)

### Social Media
<!-- Add icon library -->
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.13.1/css/all.css">

<!-- Add font awesome icons -->
<p>
    <img src="https://github.com/esa-prl/ExoMy/wiki/images/social_media_icons/discord-brands.svg" width="20px">
    <a href="https://discord.gg/gZk62gg"> Join the Community!</a>  
</p>
<p>
    <img src="https://github.com/esa-prl/ExoMy/wiki/images/social_media_icons/twitter-square-brands.svg" width="20px">
    <a href="https://twitter.com/exomy_rover"> @ExoMy_Rover</a> 
</p>
<p>
    <img src="https://github.com/esa-prl/ExoMy/wiki/images/social_media_icons/instagram-square-brands.svg" width="20px">
    <a href="https://www.instagram.com/exomy_rover/"> @ExoMy_Rover</a>
</p>

