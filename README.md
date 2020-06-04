# waymo-rosbag-record
tool to convert waymo open dataset to rosbag


## Software Requirements
- Docker ce ([Installation guide](https://docs.docker.com/engine/install/ubuntu/))


## Usage

Clone the entire repo
```bash
cd ~ 
git clone https://github.com/biomotion/waymo-rosbag-record && cd waymo-rosbag-record
```

Pull the docker image for recording the rosbag
```bash
docker pull biomotion/waymo-viewer:1.0
```

Or, you can build one for yourself.
```bash
cd ~/waymo-rosbag-record/Docker
source build.sh
```
You can grab a coffee before docker build done building new image

Now, you are able to run the docker with the script.
What `docker_run.sh` does is to mount your folder into docker container so that your tfrecord files can be processed.
```bash
./docker_run.sh [path_to_your_tfrecord_files] [path_to_your_rosbag_to_be_output]
```

For the first time you run the docker, remember to make the package for recording.
```bash
catkin_make
```

After catkin make is done, you finally can run the roslaunch command.
```bash
source devel/setup.bash
roslaunch waymo_viewer recorder.launch
```
