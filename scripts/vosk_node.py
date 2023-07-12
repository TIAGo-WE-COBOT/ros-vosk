#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic 

# This is a node that intergrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listenning to the audio stream so it won't listen to it self :)

# It publishes to the topic speech_recognition/vosk_result a custom "speech_recognition" message
# It publishes to the topic speech_recognition/final_result a simple string
# It publishes to the topic speech_recognition/partial_result a simple string


import os
import sys
import json
import queue
import numpy as np
import vosk
import sounddevice as sd
from mmap import MAP_SHARED

import rospy
import rospkg
from ros_vosk.msg import SpeechRecognition
from std_msgs.msg import String, Bool

import vosk_ros_model_downloader as downloader


class VoskSpeechRecognition():
    def __init__(self):
        ##################
        ### Queue init ###
        ##################
        # Define a queue to buffer incoming audio data. 
        self.q = queue.Queue()

        self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        if self.input_dev_num == -1:
            rospy.logfatal('No input device found')
            raise ValueError('No input device found, device number == -1')

        # Get information on the device and set sample rate accordingly
        device_info = sd.query_devices(self.input_dev_num, 'input')
        self.samplerate = int(device_info['default_samplerate'])    # soundfile expects an int,
                                                                    # sounddevice provides a float
        rospy.set_param('vosk/sample_rate', self.samplerate)

        #######################
        ### Recognizer init ###
        #######################
        # Params set from yaml file in `cfg`
        lang_model_name = rospy.get_param('vosk/model', "vosk-model-small-en-us-0.15")
        spk_model_name  = rospy.get_param('vosk/spk_model', "vosk-model-spk-0.4")
        
        rospack = rospkg.RosPack()
        rospack.list()
        package_path = rospack.get_path('ros_vosk')
        models_dir = os.path.join(package_path, 'models')

        # Get the language-specific model for transcription (if not already downloaded)
        lang_model_path = os.path.join(models_dir, lang_model_name)
        self._get_model(lang_model_path, models_dir)
        
        if not rospy.has_param('vosk/model'):
            rospy.set_param('vosk/model', lang_model_name)

        # Get the language-agnostic model for speaker recognition (if not already downloaded)
        spk_model_path = os.path.join(models_dir, spk_model_name)
        self._get_model(spk_model_path, models_dir)

        if not rospy.has_param('vosk/spk_model'):
            rospy.set_param('vosk/spk_model', spk_model_name)
        
        vosk.SetLogLevel(-1)    # suppress the verbose Vosk log
        lang_model = vosk.Model(lang_model_path)
        spk_model  = vosk.SpkModel(spk_model_path)

        # GPUInit automatically selects a CUDA device and allows multithreading.
        gpu = vosk.GpuInit() #TODO. Check if it works

        self.rec = vosk.KaldiRecognizer(lang_model, self.samplerate)
        self.rec.SetSpkModel(spk_model)

        ###################
        ### Stream init ###
        ###################
        self.timer = 0.0            #[s]
        self.t_old = rospy.Time(0)  #[s]
        self.TIME_THRESHOLD = 2.0   #[s]
        # TODO. This should be removed and data got directly from audio topic
        self.stream = sd.RawInputStream(samplerate=self.samplerate, 
                                        blocksize=16000, 
                                        device=self.input_dev_num, 
                                        dtype='int16',
                                        channels=1)

        #####################
        ### ROS node init ###
        #####################
        # Subscribe to the TTS (if any) to avoid the system listening to itself.
        self.tts_status = False
        self.tts_status_listener = rospy.Subscriber('/tts/status', 
                                                    Bool, 
                                                    self.tts_get_status)
        # Get ready to broadcast the results.
        self.pub_vosk = rospy.Publisher('speech_recognition/vosk_result',
                                        SpeechRecognition, 
                                        queue_size=10)
        self.pub_final = rospy.Publisher('speech_recognition/final_result', 
                                         String, 
                                         queue_size=10)
        self.pub_partial = rospy.Publisher('speech_recognition/partial_result',
                                           String, queue_size=10)
        self.msg = SpeechRecognition()

        rospy.on_shutdown(self.cleanup)

    def cleanup(self):
        self.stream.abort()
        rospy.logwarn("Shutting down VOSK speech recognition node...")
       
    def start_stream(self):
        self.stream.start()
        rospy.logdebug('Started recording')

    def tts_get_status(self,msg):
        self.tts_status = msg.data


    def speech_recognize(self):
        # Assume no detection if not explicitly found
        partial_text = 'unk'
        final_text   = 'unk'
        spk_sig      = []
        
        buffer, overflowed = self.stream.read(self.stream.read_available)
        if overflowed:
            rospy.logwarn('Audio buffer overflowed. Some data have been lost.')
        self.q.put(bytes(buffer))

        if self.tts_status == True:
            # If the text to speech is operating, clear the queue
            with self.q.mutex:
                self.q.queue.clear()
            self.rec.Reset()

        elif self.tts_status == False:
            data = self.q.get()
            if self.rec.AcceptWaveform(data):

                # In case of final result
                result_final = self.rec.FinalResult()
                final_dict = json.loads(result_final)
                lentext = len(final_dict["text"])
                if lentext > 2:
                    final_text = final_dict["text"]
                    rospy.loginfo(final_text)
                    
                    if "spk" in final_dict:
                        spk_sig = final_dict["spk"]
                        
                # Resets current results so the recognition can continue from scratch
                self.rec.Reset()
            
            else: # NOTE. Do we ever enter here?
                # In case of partial result
                result_partial = self.rec.PartialResult()
                if (len(result_partial) > 20):

                    partial_dict = json.loads(result_partial)
                    partial_text = partial_dict["partial"]

            # Broadcast the results
            if final_text != 'unk' or partial_text != 'unk':
                self.format_msg(final=final_text,
                                partial=partial_text,
                                spk_sig=spk_sig)
                self.pub_vosk.publish(self.msg)
                rospy.sleep(0.1)

            if final_text != 'unk':
                self.pub_final.publish(final_text)
                rospy.sleep(0.1)
            elif partial_text != 'unk':
                self.pub_partial.publish(partial_text)


    def format_msg(self, final = 'unk', partial = 'unk', spk_sig = []):
        self.msg.header.stamp = rospy.Time.now()
        self.msg.final_result = final
        self.msg.partial_result = partial
        self.msg.spk_xvector = spk_sig
        #self.msg.is_speech_recognized = 
        

    @staticmethod
    def _cosine_dist(x, y): # currently unused
        """ Get the cosine distances between two arrays. Here `x` and `y` are supposed to be two x-vectors. Thus the distance accounts for the similarity between two voices. """
        nx = np.array(x)
        ny = np.array(y)
        return 1 - np.dot(nx, ny) / np.linalg.norm(nx) / np.linalg.norm(ny)


    @staticmethod
    def _get_model(model_path, models_dir):
        """ Check if the model at the given path exists. If not, call the downloader to prompt the user to download the model via GUI. """
        if not os.path.exists(model_path):
            print (f"model '{model_name}' not found in '{models_dir}'! Please use the GUI to download it or configure an available model...")
            model_downloader = downloader.model_downloader()
            model_downloader.execute()
            model_name = model_downloader.model_to_download


if __name__ == '__main__':
    rospy.init_node('vosk_node', anonymous=False)

    asr = VoskSpeechRecognition()
    rate = rospy.Rate(10)
    try:
        asr.start_stream()
        while not rospy.is_shutdown():
            asr.speech_recognize()
            rate.sleep()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the vosk speech recognition node...")
        rospy.sleep(1)
        print("node terminated")
