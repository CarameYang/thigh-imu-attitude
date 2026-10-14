"""状态机安全门与回放一致性测试。 / State gating and replay consistency tests."""
import json
import unittest
from zupt_offline import ROOT,load
from train_zupt import detect
from stationary_stream import StationaryDetector

MODEL=ROOT/'analysis/zupt_training_20261007_164450_519158/model.json'


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.model=json.loads(MODEL.read_text())
        self.d=StationaryDetector(self.model)

    def test_confirm_motion_and_timeout(self):
        # 从静止进入确认，再确认静止；强运动和超时必须撤销。
        # Enter confirmation before stationary; strong motion and timeout must revoke permission.
        states=[self.d.push(i*20000,i,[0,0,9.80665],self.model['bias_radps']) for i in range(50)]
        self.assertFalse(states[10]['zupt_allowed'])
        self.assertTrue(states[-1]['zupt_allowed'])
        self.assertFalse(self.d.push(1000000,50,[0,0,9.80665],[5,0,0])['zupt_allowed'])
        self.assertFalse(self.d.no_data()['zupt_allowed'])

    def test_gap_and_invalid_clear_confirmation(self):
        for i in range(50):self.d.push(i*20000,i,[0,0,9.80665],self.model['bias_radps'])
        self.assertEqual(self.d.push(1020000,51,[0,0,9.80665],[0,0,0])['state'],'unknown')
        self.assertEqual(self.d.push(1040000,52,[float('nan'),0,0],[0,0,0])['state'],'unknown')

    def test_offline_feature_equivalence(self):
        # 所有录制逐包回放：特征匹配原模型，许可仅能出现在候选窗口。
        # Replay all recordings: features match offline computation and permission implies candidacy.
        for folder in sorted((ROOT/'recordings').iterdir()):
            if not (folder/'packets.jsonl').exists():continue
            rows=load(folder);self.d.reset()
            for r,expected in zip(rows,detect(self.model,rows)):
                actual=self.d.push(r['us'],r['seq'],r['a'],r['w'])
                if expected['features'] is None:self.assertIsNone(actual['features'])
                else:
                    for a,b in zip(actual['features'],expected['features']):self.assertAlmostEqual(a,b,places=10)
                if actual['zupt_allowed']:self.assertEqual(expected['state'],'candidate')


if __name__=='__main__':unittest.main()
