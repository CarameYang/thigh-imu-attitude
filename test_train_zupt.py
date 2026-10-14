"""训练隔离与区间边界测试。 / Training isolation and interval-boundary tests."""
import copy
import json
import unittest
from train_zupt import ROOT, check_split, segment, fit, evaluate, detect
from zupt_offline import load


class TrainingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.split=json.loads((ROOT/'zupt_split.json').read_text(encoding='utf-8'))
        cls.data={s['session']:load(ROOT/'recordings'/s['session'])
                  for role in ('training','validation') for s in cls.split[role]}

    def test_overlap_rejected(self):
        split=copy.deepcopy(self.split)
        split['validation'].append(dict(split['training'][0]))
        with self.assertRaises(ValueError):check_split(split)

    def test_validation_changes_do_not_affect_training(self):
        # 任意修改验证测量，模型必须保持完全一致。
        # Arbitrary validation measurement changes must leave the fitted model unchanged.
        cfg=self.split['detector']
        model=fit([(s,self.data[s['session']]) for s in self.split['training']],cfg)
        altered=copy.deepcopy(self.data)
        for s in self.split['validation']:
            for row in altered[s['session']]:
                if s['start_s']<=row['t']<s['end_s']:
                    row['w']=[100,100,100];row['a']=[100,100,100]
        other=fit([(s,altered[s['session']]) for s in self.split['training']],cfg)
        self.assertEqual(model,other)

    def test_windows_stay_inside_labels(self):
        # 窗口不得借用 5 秒前的数据，也不得跨越缺号。
        # Windows must not borrow pre-5-second samples or cross missing IDs.
        for s in self.split['validation']:
            rows=self.data[s['session']]
            _,bounds=segment(rows,s,self.split['detector'])
            for j,end in bounds:
                self.assertGreaterEqual(rows[j]['t'],5)
                self.assertLess(rows[end-1]['t'],10)
                self.assertTrue(all(rows[k]['seq']==rows[k-1]['seq']+1 for k in range(j+1,end)))

    def test_model_roundtrip_and_labels_do_not_force_prediction(self):
        cfg=self.split['detector']
        model=fit([(s,self.data[s['session']]) for s in self.split['training']],cfg)
        s=self.split['validation'][-1];rows=self.data[s['session']]
        restored=json.loads(json.dumps(model))
        self.assertEqual(detect(model,rows),detect(restored,rows))
        # 用户标签不会强制检测器输出绿色。
        # User labels do not force green detector outputs.
        altered=copy.deepcopy(rows)
        for row in altered:
            if 5<=row['t']<10:row['w']=[100,100,100]
        self.assertEqual(evaluate(restored,altered,s)['candidate_windows'],0)


if __name__=='__main__':
    unittest.main()
