#!/usr/bin/env python

from conversational_agency.openai_utils.chatter import OpenAIChatter

import rospy
from std_msgs.msg import String
from ros_vosk.msg import speech_recognition

SILENCE_THRESHOLD = 2.0 #[s]

class ChatBot():
    def __init__(self):
        # Initialize the OpenAI model to generate responses
        self.ai_chatter = OpenAIChatter()
        # Listen to Vosk speech recognition results
        self.sr_sub = rospy.Subscriber('speech_recognition/vosk_result', 
                                         speech_recognition, 
                                         self.listen
                                         )
        # Initialize the publisher to call the TTS service
        self.tts_pub = rospy.Publisher('/tts/phrase', 
                                       String, 
                                       queue_size=1
                                       )
        
        self.text = ""
        self.is_speech_recognized = False
    
    def listen(self, msg):
        self.is_speech_recognized = msg.isSpeech_recognized
        
        if msg.final_result == "unk":
            pass
        else:
            self.text += msg.final_result

    def talk(self):
        if len(self.text.split()) > 1:
            # Neglect one-word answers.
            # NOTE about the rationale: 
            # When using the italian model, and there is just background noise, often the model detects "grazie" instead. Still, some one-word answer could be very meaningful (as yes and no). TODO. Implement a better logic. 
            self.text = ""
            return
        prompt = self.ai_chatter.generate_prompt(self.text)
        ans = self.ai_chatter.generate_response(prompt)
        self.tts_pub.publish(ans)
        self.text = ""
    
if __name__ == "__main__":
    rospy.init_node('chatbot')
    cb = ChatBot()

    #st = SilenceTimer()
    silence = False

    t = rospy.Time.now()   # start the timer
    while not rospy.is_shutdown():
        if cb.is_speech_recognized:
            t = rospy.Time(0)
        if cb.text and (rospy.Time.now() - t) > rospy.Duration(SILENCE_THRESHOLD):
            cb.talk()
        
            