"""Regression for Stage 23 Webots parser failure seen on 2026-10-23.

Static check; not a replacement for parsing the world in Webots R2025a.
"""
import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class WorldFieldTests(unittest.TestCase):
    def test_no_duplicate_damping_constant_in_joint_parameter_nodes(self):
        checked=0
        for world in sorted((ROOT / 'worlds').glob('*.wbt')):
            text=world.read_text(encoding='utf-8')
            # All Stage 23 JointParameters/HingeJointParameters are one-line nodes.
            for match in re.finditer(r'(?:HingeJointParameters|JointParameters)\s*\{[^{}]*\}', text):
                fields=re.findall(r'\bdampingConstant\b', match.group())
                self.assertLessEqual(len(fields),1, f'{world.name}: duplicate dampingConstant')
                checked += 1
        self.assertGreaterEqual(checked,12)

if __name__ == '__main__': unittest.main()
