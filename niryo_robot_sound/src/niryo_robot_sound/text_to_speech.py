# Lib
from tempfile import NamedTemporaryFile

import rospy
import os
from gtts import gTTS, gTTSError

from niryo_robot_sound.srv import TextToSpeech, TextToSpeechRequest


class NiryoTextToSpeech(object):

    def __init__(self, sound_manager, sound_data_base):
        # - Init
        self.__sound_database = sound_data_base
        self.__sound_manager = sound_manager

        self.__languages = {
            TextToSpeechRequest.ENGLISH: 'en',
            TextToSpeechRequest.FRENCH: 'fr',
            TextToSpeechRequest.SPANISH: 'es',
            TextToSpeechRequest.MANDARIN: 'zh-CN',
            TextToSpeechRequest.PORTUGUESE: 'pt'
        }

        # - Services
        rospy.Service('~text_to_speech', TextToSpeech, self.__callback_text_to_speech)

    # - Callbacks
    def __callback_text_to_speech(self, req):
        if len(req.text) > 100:
            return False, "Text too long, length limited to 100 characters"
        elif req.language not in self.__languages:
            return False, "Unknown language"

        try:
            self.say(req.text, self.__languages[req.language])
        except RuntimeError as e:
            return False, str(e)
        return True, "Success"

    def say(self, text, language):
        tts = gTTS(text, lang=language)
        with NamedTemporaryFile(dir=self.__sound_database.user_sound_directory_path, suffix='.mp3') as tts_file:
            try:
                tts.write_to_fp(tts_file)
                tts_file.flush()
            except gTTSError as e:
                raise RuntimeError(f'gTTS call failed: {e}') from e

            self.__sound_database.refresh_user_sounds()
            self.__sound_manager.play_user_sound(os.path.basename(tts_file.name))
        self.__sound_database.refresh_user_sounds()
