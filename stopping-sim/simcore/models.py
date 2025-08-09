from dataclasses import dataclass
from typing import List


@dataclass
class Vehicle:
    name: str
    mass_t: float  # ton 단위 (json 기준)
    a_max: float
    j_max: float
    notches: int
    notch_accels: List[float]
    tau_cmd_ms: int
    tau_brk_ms: int


@dataclass
class Segment:
    L: float
    grade: float = 0.0


@dataclass
class SimParams:
    dt: float = 0.01
    v0: float = 25.0
    mu: float = 1.0
