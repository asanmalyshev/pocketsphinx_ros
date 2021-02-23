# ROS package for pocketsphinx  
<p align="center">

<!-- <add logo> -->

[![License](https://img.shields.io/badge/License-BSD%203--Clause-green.svg)](https://opensource.org/licenses/BSD-3-Clause)
<!-- <a><img src="https://img.shields.io/badge/ROS-Melodic-blue" alt="ros_version_melodic" /></a> -->
<a><img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ros_version_noetic" /></a>
</p>
Original repository: https://github.com/mikeferguson/pocketsphinx

Also used repository: https://github.com/gorinars/ros_voice_control  
  
More about pocketsphinx [here](https://cmusphinx.github.io/)
  
This package is an attempt to bring offline speech recognition to ROS. Pocketsphinx already offers many easy-to-use features in this domain, hence this package can be considered as an extension of pocketsphinx in the ROS world!  

## Installation
### Dependencies  
1. Install packages
For 18.04:
```shell
sudo apt-get install -y python python-dev python-pip build-essential swig libpulse-dev git libasound-dev portaudio19* bison libsphinxbase* libpocketsphinx* pocketsphinx-doc python-pyaudio python3-pyaudio pocketsphinx* sphinx*
```
For Ubuntu 20.04:
```shell
sudo apt-get install -y swig libpulse-dev libasound2 portaudio19-dev  pocketsphinx bison libsphinxbase-dev libpocketsphinx-dev python3-pyaudio pocketsphinx-en-us  python3-pocketsphinx
```
2. Clone voice recognizer repo in your workspace
```shell
cd ~/catkin_ws/src
git clone https://github.com/asanmalyshev/pocketsphinx_ros
```
### Mic setup
Use your audio manager to setup microphone and check it.
Ubuntu uses PusleAudio.

## Build
In terminal:  
```shell
cd ~/catkin_ws
catkin_make # if you use catkin_make
catkin build # if you use catkin tools 
```

## Setup
### Commands setup
in ***share/*** folder there're some dicts, key phrase and grammar files examples.

For russian:
- share/voice_cmd_ru.dic
- share/voice_cmd_ru.kwlist
- share/grammar_ru.gram

<!-- For english: -->
<!-- - share/voice_cmd_en.dic -->
<!-- - share/voice_cmd_en.kwlist -->
<!-- - share/grammar_en.gram -->

*dic* files contain words to be recognized. There're some ways to get that file:
1. In  pocketsphinx model directory (by default */usr/share/pocketsphinx/model*) find various acoustic models.
Use one of them to form your dictionary: in model's folder find dictionary file (usually it has *dict* extention),
find words you are interested in and put the whole line (word + other stuff) in your dictionary.
For standard English model, cd in */usr/share/pocketsphinx/model/en-us* and open *cmudict-en-us.dict* file.
1. For russian language [transcriptor module](https://github.com/zamiron/ru4sphinx) might me used.
    1. Download repository, cd in transcriptor module
    ```shell
    cd ~/catkin_ws
    git clone https://github.com/zamiron/ru4sphinx
    cd ru4sphinx/text2dict
    ```
    1. create a file *my_dictionary* file with some words (each word on new line):
    ```
    медведь
    берёза
    бабалайка
    ```
    1. get dict file:
    ```shell
    perl dict2transcript.pl my_dictionary my_dictionary.dic
    ```

*kwlist* files are so called keyphrase files. They contain phrases to be highlited by pocketsphinx.
Phrases must be composed from words in dict. List contains phrases and their thresholds.
For shorter keyphrases you can use smaller thresholds like 1e-1, for longer keyphrases the threshold must be bigger, up to 1e-50.
Line pattern (a number between slashes is phrase threshold):
```
BACK /1e-5/ 
```
*gram* files contain grammar in jsgf format. Describe your phrases in jsgf using words from *dic*

Read more on [official page](https://cmusphinx.github.io/wiki/tutoriallm/)

<!-- ### Setup check -->
<!-- Just to verify everything is set up correcty, run in console: -->
<!-- ```shell -->
<!-- roscd pocketsphinx_ros/share -->
<!-- pocketsphinx_continuous -inmic yes -hmm /usr/share/pocketsphinx/model/en-us/en-us/ -dict share/voice_cmd_en.dic -kws share/voice_cmd_en.kwlist  -->
<!-- ``` -->
<!-- Say some words from dictionary. In that test only pocketsphinx program is used. -->

### Another language acoustic models
Embedded acoustic model is quite poor. New models might be found on [project site](https://cmusphinx.github.io/wiki/download/).
Look for Download link in model section. There're a lot of models in different languages. 
As in *share/* folder there're files for Russian language, let's add a russian acoustic model.
- look for a model using link above;
- download it;
- unzip / untar in in */usr/share/pocketsphinx/model/* (optionally rename it in *ru*);
- launch package with new params

```shell
cd /usr/share/pocketsphinx/model
sudo wget 'https://sourceforge.net/projects/cmusphinx/files/Acoustic%20and%20Language%20Models/Russian/cmusphinx-ru-5.2.tar.gz'
sudo mkdir ru && sudo tar -xzvf cmusphinx-ru-5.2.tar.gz --directory ru --strip-components=1
#check if in ru/ there are some files. If any - remove downlowaded archive
sudo rm cmusphinx-ru-5.2.tar.gz
```
***Caution: In example above wget utility was used. It doesn't guarantee downloading the latest version of library. Use link to get the latest one.***

## Launch
### kws mode:
```shell
roslaunch pocketsphinx_ros decoder_kws.launch
```
### jsgf mode:
```shell
roslaunch pocketsphinx_ros decoder_jsgf.launch
```
### Nodes params
| name | description | default value
| --- | --- | ---
| hmm | path to folder of acoustic model | /usr/share/pocketsphinx/model/ru
| dict  | dictionary file | $(find pocketsphinx_ros)/share/voice_cmd_ru.dic
| kws   | kwlist file | $(find pocketsphinx_ros)/share/voice_cmd_ru.kwlist
| gram   | gram file | $(find pocketsphinx_ros)/share/grammar_ru.gram
| grammar   | using grammar (defined in gram file) | robot
| rule   | rule to recognize phrases (defined in gram file) | cmd

### Topics info
#### decoder Publisher
| topic | type | description
| --- | --- | ---
| /decoded_phrase | pocketsphinx_ros/DecodedPhrase | recognized commands
 
#### decoder Subscriber
| topic | type | description
| --- | --- | ---
| /sphinx_audio | [std_msgs/UInt8MultiArray](http://docs.ros.org/kinetic/api/std_msgs/html/msg/UInt8MultiArray.html) | processing audio
 
