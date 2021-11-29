# Lib
import rospy
import os
from gtts import gTTS

from niryo_robot_sound.srv import TextToSpeech, TextToSpeechRequest


class NiryoTextToSpeech(object):

    def __init__(self, sound_manager, sound_data_base):
        # - Init
        self.__sound_database = sound_data_base
        self.__sound_manager = sound_manager

        self.__languages = {TextToSpeechRequest.ENGLISH: 'en', TextToSpeechRequest.FRENCH: 'fr',
                            TextToSpeechRequest.SPANISH: 'es', TextToSpeechRequest.MANDARIN: 'zh-CN',
                            TextToSpeechRequest.PORTUGUESE: 'pt'}

        self.__tts_name = 'last_text_to_speech.mp3'

        # - Services
        rospy.Service('~text_to_speech', TextToSpeech, self.__callback_text_to_speech)

    # - Callbacks
    def __callback_text_to_speech(self, req):
        if len(req.text) > 100:
            return False, "Text too long, length limited to 100 characters"
        elif req.language not in self.__languages:
            return False, "Unknown language"

        self.say(req.text, self.__languages[req.language])
        return True, "Success"

    def say(self, text, language):
        tts = gTTS(text, lang=language)
        sound_path = os.path.join(self.__sound_database.user_sound_directory_path, self.__tts_name)
        tts.save(sound_path)
        self.__sound_database.refresh_user_sounds()
        self.__sound_manager.play_user_sound(self.__tts_name)
