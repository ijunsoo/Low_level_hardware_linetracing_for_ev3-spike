# 강화학습 준비 패키지 (3단계): 랜덤 트랙 생성기, 빠른 병렬 시뮬, 규칙기반 병렬판, 학습 환경.
# sim/ 의 core를 그대로 쓰므로 sim/ 을 import 경로에 올린다.
import os
import sys

_SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SIM_DIR not in sys.path:
    sys.path.insert(0, _SIM_DIR)
