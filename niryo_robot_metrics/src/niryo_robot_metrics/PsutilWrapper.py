# coding=utf-8
import psutil

from .GenericWrapper import GenericWrapper


class PsutilWrapper(GenericWrapper):
    __available_metrics__ = [
        'cpu_usage',
        'cpu_temperature',
        'ram_usage',
        'rom_usage',
        'eth0_address',
        'eth0_netmask',
        'eth0_is_used',
        'wlan0_address',
        'wlan0_netmask',
        'wlan0_is_used',
    ]

    __ETH_IFACE = 'eth0'
    __WIFI_IFACE = 'wlan0'

    def _fetch_datas(self):
        self._data.update({
            'cpu_usage': {
                'name': 'cpu_usage',
                'value': self.get_cpu_usage(),
                'unit': '%',
            },
            'cpu_temperature': {
                'name': 'cpu_temperature',
                'value': self.get_cpu_temperature(),
                'unit': '°C',
            },
            'ram_usage': {
                'name': 'ram_usage',
                'value': self.get_ram_usage(),
                'unit': '%',
            },
            'rom_usage': {
                'name': 'rom_usage',
                'value': self.get_rom_usage(),
                'unit': '%',
            },
        })
        for iface, iface_data in self.get_net_metrics().items():
            for key, value in iface_data.items():
                name = '{}_{}'.format(iface, key)
                self._data[name] = {'name': name, 'value': value, 'unit': None}

    @staticmethod
    def get_cpu_usage():
        return psutil.cpu_percent()

    @staticmethod
    def get_cpu_temperature():
        return list(psutil.sensors_temperatures().values())[0][0].current

    @staticmethod
    def get_ram_usage():
        return psutil.virtual_memory().percent

    @staticmethod
    def get_rom_usage():
        return psutil.disk_usage('/').percent

    @classmethod
    def get_net_metrics(cls):
        network_config = psutil.net_if_addrs()
        network_usage = psutil.net_if_stats()
        network = {
            cls.__WIFI_IFACE: {},
            cls.__ETH_IFACE: {},
        }
        try:
            for iface in network.keys():
                for address in network_config[iface]:
                    # filter out ipv6 addresses
                    if ':' not in address.address:
                        network[iface]['address'] = address.address
                        network[iface]['netmask'] = address.netmask
                        network[iface]['is_used'] = network_usage[iface].isup
        except KeyError:
            return {}
        return network
