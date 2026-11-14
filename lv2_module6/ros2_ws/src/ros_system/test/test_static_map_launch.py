#!/usr/bin/env python3
"""Static-map launch contracts. Constructs actions only, never starts robot nodes."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]

def load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class StaticMapTests(unittest.TestCase):
    def setUp(self):
        self.mod = load(ROOT / 'launch/ros_system.launch.py')
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        (self.folder / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
        self.map = self.folder / 'map.yaml'
        self.map.write_text('image: map.pgm\nresolution: 0.05\norigin: [-1.2, 2.3, 0.4]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    def context(self, **args):
        ctx = LaunchContext()
        for e in self.mod.generate_launch_description().entities:
            if isinstance(e, DeclareLaunchArgument):
                ctx.launch_configurations[e.name] = perform_substitutions(ctx, e.default_value)
        ctx.launch_configurations.update(args)
        return ctx

    def active(self, ctx):
        out = {}
        for action in self.mod.slam_nodes():
            for e in (action.actions if isinstance(action, TimerAction) else [action]):
                if isinstance(e, Node) and (e.condition is None or e.condition.evaluate(ctx)):
                    exe = e.node_executable
                    out[e._Node__node_name or (exe if isinstance(exe, str) else perform_substitutions(ctx, exe))] = e
        return out

    def test_static_keeps_localization_removes_reconstruction(self):
        ctx = self.context(map_backend='static_2d', static_map_yaml=str(self.map))
        self.assertEqual(set(self.active(ctx)), {'cuvslam_slam', 'map_server', 'lifecycle_manager_map', 'rewind', 'static_esdf_slice'})
        with patch.object(self.mod.importlib.util, 'find_spec', return_value=object()), patch.object(self.mod, 'get_package_share_directory', return_value='/installed') as pkg:
            for action in self.mod.check_slam(ctx):
                action.execute(ctx)
            pkg.assert_called_once_with('nav2_map_server')
        server = self.active(ctx)['map_server']
        params = evaluate_parameters(ctx, server._Node__parameters)[0]
        self.assertEqual(params['yaml_filename'], str(self.map))
        self.assertEqual(params['frame_id'], 'map')
        manager = self.active(ctx)['lifecycle_manager_map']
        params = evaluate_parameters(ctx, manager._Node__parameters)[0]
        self.assertTrue(params['autostart'])
        self.assertEqual(list(params['node_names']), ['map_server'])

    def test_original_backend_and_no_map(self):
        # nvblox 기본 — pole_depth_gate · nvblox_map · rewind는 py_helpers 한 프로세스 (#214)
        self.assertEqual(set(self.active(self.context())), {'cuvslam_slam', 'nvblox_node', 'py_helpers'})
        self.assertEqual(set(self.active(self.context(py_helpers_one_process='false'))),
                         {'cuvslam_slam', 'nvblox_node', 'nvblox_map', 'pole_depth_gate', 'rewind'})
        self.assertEqual(set(self.active(self.context(use_slam='false', map_backend='static_2d'))), {'rewind'})

    def test_rejects_missing_map_and_mapping(self):
        for args in ({'map_backend': 'static_2d', 'localization': 'false'},
                     {'map_backend': 'static_2d', 'static_map_yaml': '/missing/map.yaml'},
                     {'map_backend': 'typo'}):
            with self.assertRaises(RuntimeError):
                self.mod.check_slam(self.context(**args))
        self.map.write_text('image: missing.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
        with self.assertRaises(RuntimeError):
            self.mod.validate_static_map(str(self.map))
        self.map.write_text('image: map.pgm\nresolution: .nan\norigin: [0, 0, 0]\n')
        with self.assertRaises(RuntimeError):
            self.mod.validate_static_map(str(self.map))

    def test_bringup_selects_yaml_only_map_and_forwards(self):
        mod = load(ROOT.parent / 'bringup/launch/bringup.launch.py')
        map_dir = self.folder / 'maps'
        (map_dir / 'default').mkdir(parents=True)
        (map_dir / 'default' / 'map.yaml').write_text(self.map.read_text())
        args = dict(map='saved', fake_slam='false', use_ros_system='true', map_dir=str(map_dir), map_name='default', map_backend='static_2d',
                    use_gripper_cam='false', use_mission='true', perception='auto', use_perception='auto', nvblox_params_file='')
        ctx = LaunchContext()
        ctx.launch_configurations.update(args)
        with patch.object(mod, 'get_package_share_directory', return_value='/installed'):
            for action in mod.setup_map(ctx):
                action.execute(ctx)
        self.assertEqual(ctx.launch_configurations['map_name'], 'default')
        self.assertEqual(ctx.launch_configurations['static_map_yaml'], str(map_dir / 'default' / 'map.yaml'))
        missing_name = LaunchContext()
        missing_name.launch_configurations.update({**args, 'map_name': ''})
        with self.assertRaises(RuntimeError):
            mod.setup_map(missing_name)
        self.assertEqual(ctx.launch_configurations['use_real_slam'], 'true')
        self.assertEqual(ctx.launch_configurations['localization'], 'true')
        for updates in ({'map': 'slam'}, {'fake_slam': 'true'}, {'map_name': 'missing'}):
            ctx = LaunchContext(); ctx.launch_configurations.update({**args, **updates})
            with self.assertRaises(RuntimeError):
                mod.setup_map(ctx)

if __name__ == '__main__':
    unittest.main()
