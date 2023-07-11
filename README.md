ros_vosk
======================

A ROS package for speech-to-text services based on [Vosk](https://github.com/alphacep/vosk-api)

## Index
- [Installation](#installation)
  - [OPTIONAL. Setup a virtual environment for Vosk](#optional-setup-a-virtual-environment-for-vosk)
- [How to use](#how-to-use)
  - [Change the model for _speech recognition_ and/or _text-to-speech_](#change-the-model-for-speech-recognition-andor-text-to-speech)
    - [Create a config](#create-a-config)
    - [Use a config](#use-a-config)
- [Interface](#interface)
- [Simulating microphone input](#simulating-microphone-input)
  - [To install `pavucontrol`](#to-install-pavucontrol)
  - [To use `pavucontrol`](#to-use-pavucontrol)
- [TODO](#todo)
- [Authors](#authors)
- [Contributors](#contributors)

## Installation

1. Install this package and vosk

  ```bash
  sudo apt install ros-${ROS_DISTRO}-ros-vosk
  ```
  don't forget to run `catkin_make` (or `catkin build`, depending on how you built your workspace).
  
2. Install Dependencies

  If using ROS MELODIC run first: 
  ```bash
  sudo apt install python3-pip python3-yaml
  ```
  Then run for MELODIC & NOETIC:
  ```bash
  pip3 install sounddevice
  pip3 install vosk
  ``` 
  And if you want to use the TTS engine please run:
  ```bash
  sudo apt install espeak
  pip install pyttsx3
  ```  

### OPTIONAL. Setup a virtual environment for Vosk

> :warning: If you are not familiar with virtual environments, you might struggle to follow this part, and to later make everything work (especially if you plan to add your own features to the package). Choose wisely! 

- Create a virtual environment
```
python -m venv /path/to/new/virtual/environment
```

- Activate the virtual environment
```
source /path/to/new/virtual/environment/bin/activate
```

- Follow the [Installation](#installation) steps. 
> :memo: Packages installed with `apt`` will be installed system-wide anyway. Packages installed with `pip` will be installed in the virtual environment.

- Define an environment variable called `VOSK_VENV`` with the path to the Python interpreter.

> :warning: If running a node with `rosrun`, you will revert to the system Python interpreter if not specified differently. If you want to run it in the same virtual environment you defined above, you should create a dummy launch file, so to run the node with the Vosk venv interpreter as launch prefix, or to use the `--prefix` argument to point at the interpreter location.

## How to use 

Launch the speech recognition node

```bash
roslaunch ros_vosk ros_vosk.launch
```

or by running:
```bash
rosrun ros_vosk vosk_node.py
```

::warning:: If you have followed the Setup a Virtual Environment for Vosk instructions, then you need to run
```bash
rosrun --prefix </absolute/path/to/the/venv/interpreter> ros_vosk vosk_node.py
```
in order to run your node with the proper interpreter.

### Change the model for _speech recognition_ and/or _text-to-speech_

#### Create a config
You can specify the model you want to use by adding a `.yaml` file in the `cfg` folder. You can use the [template config file](/cfg/_template.yaml) for reference.

#### Use a config
The config file can be then used by specifying it at launch with the `config` argument. For example, use the following to launch the `vosk_node` with the configuration specified in `cfg/it.yaml` (for Italian STT and TTS):
```bash
roslaunch ros_vosk vosk_node config:=it
```
> :warning: The name of the configuration file must be given to the `config` argument <ins>without</ins> the `.yaml` extension.

If the model you want to use is not already present in your local `models` folder, a dialog window will pop-up to enable you to download it. From the second use on, you will not need this extra step anymore.

### Simulating microphone input
In some situations, testing `ros_vosk` with real live recordings is not the best option. For instance, you might want to test your code against the same conversation several times, either for debugging a specific issues, or for results reproducibility. Moreover, you might not want to bother your colleagues by repeatedly asking them to talk with you, or you might be working alone when you are in need of a multi-speaker conversation.

In all the above scenarios, you might want to reproduce an audio file (e.g. a previously recorded conversation), and let `ros_vosk` thing that the sound is actually coming from the microphone right now. Though more elegant and automatized solutions may exists (please tell me!) in the following you will find instructions on how to install and use `pavucontrol` to achieve such a result.

#### To install `pavucontrol`
```
sudo apt-get update
sudo apt-get install pavucontrol
```

#### To use `pavucontrol`

Start the `vosk_node` and the audio file you want to send as _fake_ microphone input (in your favorite player, as VLC). Then in another terminal run
```
pavucontrol
```
In the window that pops up, go in the _Recording_ tab and spot the entry associated to your node (it will likely be the onely one, so not too difficult). In the drop-down menù, select the _Monitor of ..._ item. The `vosk node` should now be "listening" to the audio file as it was a microphone input!

## Interface

### Publishing Topics
* speech_recognition/vosk_result    -> vosk_node.py publishes a custom "speech_recognition" message
* speech_recognition/final_result   -> vosk_node.py publishes a simple string with the final result
* speech_recognition/partial_result -> vosk_node.py publishes a simple string with the partial result
* tts/status -> tts_engine.py publishes the state of the engine. True if it is speaking False if it is not. If the status is true vosk_node won't process the audio stream so it won't listen to itself 
* tts/phrase -> tts_engine.py subscribes to this topic in order to speak the given string. Name your desire and it shall be heard by all in the room..

## TODO
- [x] languages
  - [x] support for Italian model
- [ ] speaker recognition
  - [x] support for speaker recognition model
  - [ ] speaker tracking based on x-vector
  - [ ] speaker recognition based on sound direction (for multi-channel microphones)
- [ ] make the output (partially) ROS4HRI-compliant
- [ ] msg should be camel-case
- [ ] make it compatible with `audio_common` (i.e. remove `sounddevice` dependency)
- [ ] add instruction on how to use `pavucontrol` to simulate microphone input
  - [ ] add GUI screenshot(s)


## Authors
Angelo Antikatzidis <an.antikatzidis@gmail.com>
Nickolay V. Shmyrev <nshmyrev@gmail.com>

## Contributors
Luca Pozzi <luca6.pozzi@mail.polimi.it>