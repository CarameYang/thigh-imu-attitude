"""离线检测的关键行为测试。 / Tests for key offline detector behavior."""
import copy
import unittest
from zupt_offline import analyze, load, windows, DEFAULT, G


class DetectorTests(unittest.TestCase):
    def test_gap_restarts_window(self):
        # 缺一个序号也必须重启窗口，即使时间间隔未超过上限。
        # One missing sequence ID restarts the window even below the time-gap limit.
        rows=[dict(t=i*.02,seq=i+(1 if i>=20 else 0)) for i in range(50)]
        bounds=windows(rows)
        self.assertIsNotNone(bounds[19])
        self.assertTrue(all(b is None for b in bounds[20:30]))
        self.assertEqual(bounds[30][0],20)

    def test_long_gap_restarts_window(self):
        # 时间跳跃但序号连续也不能跨越窗口。
        # A time gap with consecutive IDs must also restart the window.
        rows=[dict(t=i*.02+(1 if i>=20 else 0),seq=i) for i in range(50)]
        self.assertTrue(all(b is None for b in windows(rows)[20:30]))

    def test_validation_does_not_train(self):
        # 改变验证区间不能改变拟合偏置或阈值。
        # Altering validation data must not change fitted bias or thresholds.
        rows=load(DEFAULT)
        first,_=analyze(rows)
        altered=copy.deepcopy(rows)
        for r in altered:
            if r['t']>=55:r['w']=[v+1 for v in r['w']]
        second,_=analyze(altered)
        self.assertEqual(first['bias_radps'],second['bias_radps'])
        self.assertEqual(first['thresholds'],second['thresholds'])
        self.assertEqual(second['validation']['candidate_fraction'],0)

    def test_known_stationary_and_rotation(self):
        # 确定性样本验证静止与明显转动，不把输出当作真实步态准确率。
        # Deterministic samples verify rest versus rotation, not real gait accuracy.
        rows=[dict(t=i*.02,seq=i,a=[0,0,G],w=[.02 if not 15<=i*.02<50 else 1,0,0]) for i in range(3001)]
        report,result=analyze(rows)
        self.assertAlmostEqual(report['bias_radps'][0],.02)
        self.assertAlmostEqual(report['validation']['bias_mean_norm_after'],0)
        self.assertEqual(report['validation']['candidate_fraction'],1)
        self.assertTrue(all(r['state']=='moving' for r in result if 16<=r['t']<49))


if __name__=='__main__':
    unittest.main()
