import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import MagicMock


class _FakeNode:
	def __init__(self, *_args, **_kwargs):
		self._logger = MagicMock()

	def create_subscription(self, *_args, **_kwargs):
		return None

	def get_logger(self):
		return self._logger

	def destroy_node(self):
		return None


class _FakeUInt8MultiArray:
	pass


def _load_coms_module(monkeypatch):
	rclpy_mod = types.ModuleType('rclpy')
	rclpy_node_mod = types.ModuleType('rclpy.node')
	rclpy_node_mod.Node = _FakeNode
	rclpy_mod.node = rclpy_node_mod
	monkeypatch.setitem(sys.modules, 'rclpy', rclpy_mod)
	monkeypatch.setitem(sys.modules, 'rclpy.node', rclpy_node_mod)

	std_msgs_mod = types.ModuleType('std_msgs')
	std_msgs_msg_mod = types.ModuleType('std_msgs.msg')
	std_msgs_mod.UInt8MultiArray = _FakeUInt8MultiArray
	std_msgs_msg_mod.UInt8MultiArray = _FakeUInt8MultiArray
	std_msgs_mod.msg = std_msgs_msg_mod
	monkeypatch.setitem(sys.modules, 'std_msgs', std_msgs_mod)
	monkeypatch.setitem(sys.modules, 'std_msgs.msg', std_msgs_msg_mod)

	paho_mod = types.ModuleType('paho')
	paho_mqtt_mod = types.ModuleType('paho.mqtt')
	paho_mqtt_client_mod = types.ModuleType('paho.mqtt.client')
	paho_mqtt_client_mod.Client = MagicMock()
	paho_mqtt_mod.client = paho_mqtt_client_mod
	paho_mod.mqtt = paho_mqtt_mod
	monkeypatch.setitem(sys.modules, 'paho', paho_mod)
	monkeypatch.setitem(sys.modules, 'paho.mqtt', paho_mqtt_mod)
	monkeypatch.setitem(sys.modules, 'paho.mqtt.client', paho_mqtt_client_mod)

	ament_mod = types.ModuleType('ament_index_python')
	ament_packages_mod = types.ModuleType('ament_index_python.packages')

	def _missing_share(_name):
		raise RuntimeError('not set in test')

	ament_packages_mod.get_package_share_directory = _missing_share
	ament_mod.packages = ament_packages_mod
	monkeypatch.setitem(sys.modules, 'ament_index_python', ament_mod)
	monkeypatch.setitem(sys.modules, 'ament_index_python.packages', ament_packages_mod)

	module_path = Path(__file__).resolve().parents[1] / 'loon-e-coms' / 'coms.py'
	spec = importlib.util.spec_from_file_location('coms_module', module_path)
	module = importlib.util.module_from_spec(spec)
	spec.loader.exec_module(module)
	return module


def test_mqtt_connects_using_yaml_config(monkeypatch, tmp_path):
	module = _load_coms_module(monkeypatch)

	config_dir = tmp_path / 'config'
	config_dir.mkdir()
	config_file = config_dir / 'coms.yaml'
	config_file.write_text(
		'\n'.join([
			'mqtt:',
			'  broker:',
			'    host: "127.0.0.1"',
			'    port: 1884',
			'  topics:',
			'    destination: "loon-e/destination"',
			'    pwm: "loon-e/pwm"',
			'    path: "loon-e/path"',
			'    map: "loon-e/map"',
			'    objects: "loon-e/objects"',
			'    locations: "loon-e/locations"',
			'    obstacles: "loon-e/obstacles"',
			'  client:',
			'    id: "test-client"',
			'    keepalive: 45',
			'    qos: 1',
			'',
		]),
		encoding='utf-8',
	)

	fake_client = MagicMock()
	monkeypatch.setattr(module.mqtt, 'Client', MagicMock(return_value=fake_client))
	monkeypatch.setattr(module, 'get_package_share_directory', lambda _name: str(tmp_path))

	node = module.Communications()

	module.mqtt.Client.assert_called_once_with('test-client')
	fake_client.connect.assert_called_once_with('127.0.0.1', 1884, 45)
	fake_client.loop_start.assert_called_once()
	assert node.mqtt_qos == 1
	assert node.mqtt_topics['destination'] == 'loon-e/destination'
