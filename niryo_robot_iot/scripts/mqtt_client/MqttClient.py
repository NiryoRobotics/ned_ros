import ssl
import threading
import paho.mqtt.client as mqtt


class MqttClient:
    def __init__(self, serial, ca_file, cert_file, key_file):
        client_id = "client_{}".format(serial)
        self.__client = mqtt.Client(client_id=client_id, clean_session=False)
        self.__client.tls_set(
            ca_certs=ca_file,
            certfile=cert_file,
            keyfile=key_file,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLSv1_2,
            ciphers=None
        )

    # - Callbacks-wise functions

    def set_callbacks(self, connect_callback, disconnect_callback, message_callback):
        self.__client.on_connect = connect_callback
        self.__client.on_disconnect = disconnect_callback
        self.__client.on_message = message_callback

    def add_topic_callback(self, topic, callback_function):
        self.__client.message_callback_add(
            topic,
            lambda client, userdata, message:
                threading.Thread(target=callback_function, args=(client, userdata, message)).start()
        )

    # - Connection / Disconnection

    def connect(self, endpoint, port):
        self.__client.connect(host=endpoint, port=port)

    def disconnect(self):
        self.__client.disconnect()

    def subscribe_all(self, topics):
        self.__client.subscribe(map(lambda x: (x, 1), topics))

    def unsubscribe_all(self, topics):
        self.__client.unsubscribe(topics.keys())

    # - Loop
    def loop_start(self):
        self.__client.loop_start()

    def loop_stop(self):
        self.__client.loop_stop()

    # Getter
    @property
    def client(self):
        return self.__client
