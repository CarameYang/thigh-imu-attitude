"""大腿 IMU 逐包静止候选状态机；不推断真实位移，不执行 ZUPT。
Per-sample thigh IMU stationary state machine; no displacement inference or ZUPT update.
"""
import math
from collections import deque
from zupt_offline import features
from train_zupt import KEYS


class StationaryDetector:
    def __init__(self, model, confirmation_s=0.5):
        # 使用固定模型；确认持续时间是待实验验证的初始设置。
        # Use a fixed model; confirmation duration is provisional pending validation.
        if not math.isfinite(confirmation_s) or confirmation_s <= 0:
            raise ValueError('confirmation_s must be positive')
        self.bias=tuple(model['bias_radps'])
        self.limits=tuple(model['thresholds'][k] for k in KEYS)
        self.window=float(model['detector']['window_s'])
        self.max_gap=float(model['detector']['max_gap_s'])
        if len(self.bias)!=3 or any(not math.isfinite(x) for x in self.bias+self.limits) or any(x<=0 for x in self.limits):
            raise ValueError('Invalid bias/thresholds')
        if not 0<self.max_gap<=self.window:raise ValueError('Invalid timing parameters')
        self.confirmation=confirmation_s
        self.reset()

    def reset(self):
        self.buffer=deque();self.previous=None;self.candidate_since=None

    def push(self, timestamp_us, sequence, acceleration, angular_rate):
        # 调用端须按单节点传入 SI 测量；时间使用 MCU 测量时间。
        # Caller supplies one node's SI measurements; use MCU measurement timestamps.
        valid=(type(timestamp_us) is int and timestamp_us>=0 and type(sequence) is int and sequence>=0
               and len(acceleration)==3 and len(angular_rate)==3
               and all(type(x) in (float,int) and math.isfinite(x) for x in list(acceleration)+list(angular_rate)))
        if not valid:
            self.reset()
            return dict(state='unknown',zupt_allowed=False,reason='invalid_measurement',features=None)
        reason='warming_up'
        if self.previous is not None:
            old_t,old_seq=self.previous
            if timestamp_us<=old_t or sequence!=old_seq+1 or (timestamp_us-old_t)/1e6>self.max_gap:
                self.reset();reason='gap_or_reset'
        self.previous=(timestamp_us,sequence)
        r=dict(t=timestamp_us/1e6,a=list(acceleration),w=list(angular_rate))
        self.buffer.append(r)
        # 保留覆盖完整时间窗所需的最早样本，不连接数据间断。
        # Retain the earliest sample needed for a full time window, without bridging gaps.
        while len(self.buffer)>1 and self.buffer[1]['t']<=r['t']-self.window+1e-9:
            self.buffer.popleft()
        if r['t']-self.buffer[0]['t']<self.window-1e-6:
            return dict(state='unknown',zupt_allowed=False,reason=reason,features=None)
        f=features(list(self.buffer),self.bias)
        if not all(v<=lim for v,lim in zip(f,self.limits)):
            self.candidate_since=None
            return dict(state='not_stationary_candidate',zupt_allowed=False,reason='threshold_exceeded',features=f)
        if self.candidate_since is None:self.candidate_since=r['t']
        elapsed=r['t']-self.candidate_since
        confirmed=elapsed>=self.confirmation-1e-6
        return dict(state='stationary_candidate' if confirmed else 'confirming',zupt_allowed=confirmed,
                    reason='sustained_low_dynamics' if confirmed else 'awaiting_confirmation',
                    candidate_duration_s=elapsed,features=f)

    def no_data(self):
        # 接收端超时时必须调用，禁止沿用最后一次绿色状态。
        # Receiver must call this on timeout; never retain the last green state without fresh data.
        self.reset()
        return dict(state='unknown',zupt_allowed=False,reason='no_fresh_data',features=None)
