https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit
Update firmware: https://www.jetson-ai-lab.com/initial_setup_jon.html

https://docs.nvidia.com/sdk-manager/install-with-sdkm-jetson/index.html

Tutorials: https://developer.nvidia.com/embedded/learn/tutorials
Isaac Lab: https://developer.nvidia.com/blog/closing-the-sim-to-real-gap-training-spot-quadruped-locomotion-with-nvidia-isaac-lab/
Isaac Sim tuto: https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_workflows.html

Jetson Orin Nano Developer Kit User Guide: 
https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/index.html
Developer, getting started
https://developer.nvidia.com/embedded/learn/getting-started-jetson#tutorials

Tensor RT
https://developer.nvidia.com/tensorrt



## Memo

**PIP**
```
which pip
pip list
show pip torch
```

**Check Jetpack version**
`dpkg-query --show nvidia-l4t-core

Tensorrt : 10.3.0
`pip list`

**Models**
Onnx model zoo
https://onnx.ai/models/

**Mount Swap memory**
`sudo systemctl disable nvzramconfig`
`sudo fallocate -l 4G /mnt/4GB.swap`
`sudo mkswap /mnt/4GB.swap`
`sudo swapon /mnt/4GB.swap`

Then add the following line to the end of `/etc/fstab` to make the change persistent:

```shell
/mnt/4GB.swap  none  swap  sw 0  0
```

**Disable GUI**
sudo init 3
dorvan
6956
sudo init 5






