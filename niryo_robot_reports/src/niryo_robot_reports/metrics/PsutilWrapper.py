# coding=utf-8
import psutil

from GenericWrapper import GenericWrapper


class PsutilWrapper(GenericWrapper):
    __ETH_IFACE = 'eth0'
    __WIFI_IFACE = 'wlan0'

    def _fetch_datas(self):
        self._data += [
            {
                'name': 'cpu_usage',
                'value': self.get_cpu_usage(),
                'unit': '%',
            },
            {
                'name': 'cpu_temperature',
                'value': self.get_cpu_temperature(),
                'unit': '°C',
            },
            {
                'name': 'ram_usage',
                'value': self.get_ram_usage(),
                'unit': '%',
            },
            {
                'name': 'rom_usage',
                'value': self.get_rom_usage(),
                'unit': '%',
            },
        ]
        for iface, iface_data in self.get_net_metrics().items():
            for key, value in iface_data.items():
                self._data.append({
                    'name': '{}_{}'.format(iface, key),
                    'value': value,
                    'unit': None
                })

    @staticmethod
    def get_cpu_usage():
        return psutil.cpu_percent()

    @staticmethod
    def get_cpu_temperature():
        return psutil.sensors_temperatures().values()[0][0].current

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
