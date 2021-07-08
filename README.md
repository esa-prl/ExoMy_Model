# ExoMy - Model Repository
*Disclaimer: This model and the simulation are still in development and subject to change. If you want to contribute to the project, contact us with your plans [Discord](https://discord.gg/gZk62gg).*

This repository contains an urdf model describing Exomy.

![Urdf image](https://user-images.githubusercontent.com/10925797/98133643-20edb180-1ebe-11eb-897a-2b605b97495e.png)

## How to run
0. Clone this repository `git clone --branch ros1 https://github.com/esa-prl/ExoMy_Model ~/ExoMy_Model`
1. Checkout the branch `feature/urdf` from ExoMy_Software.
2. I had to run `xhost +SI:localuser:root` to get the GUI working.
3. Start the docker container with `sh run_exomy.sh -d`
4. Inside the docker run: `source devel/setup.sh`
5. Run: `roslaunch exomy_model display.launch gui:=True`
6. Control the joints with the gui.


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

