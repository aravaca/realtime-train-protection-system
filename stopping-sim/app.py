from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import numpy as np
import json
import os

from simcore.models import Vehicle, Segment, SimParams
from simcore.braking_curve import build_tbc
from simcore.dynamics import run_sim
from simcore.controller import TASCController

app = FastAPI()

# ✅ CORS 설정: React에서 호출 가능하도록
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 필요 시 React 도메인으로 제한
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ✅ 요청 데이터 모델 정의
class SimInput(BaseModel):
    speed_kmh: float  # 현재 속도 (km/h)
    notch: int  # 현재 노치 단계 (0~7)


@app.post("/api/run_simulation")
async def run_simulation(data: SimInput):
    # 📦 파라미터 로딩
    with open("data/vehicle.json", "r") as f:
        veh_data = json.load(f)

    veh = Vehicle(
        m=veh_data["mass_t"] * 1000,
        a_max=veh_data["a_max"],
        j_max=veh_data["j_max"],
        tau_cmd=veh_data["tau_cmd_ms"],
        tau_brk=veh_data["tau_brk_ms"],
        notches=veh_data["notches"],
        notch_accels=veh_data["notch_accels"],
    )

    seg = Segment(L=500)
    params = SimParams(v0=data.speed_kmh / 3.6, dt=0.01, mu=1.0)

    ctrl = TASCController(veh)
    ctrl.override_notch = data.notch  # <- 강제로 특정 노치 적용

    trace = run_sim(seg, veh, params, ctrl)
    tbc = build_tbc(seg.L, params.v0, veh.a_max, veh.j_max)

    # 결과를 JSON 변환
    result = {
        "trace": {
            "s": trace["s"].tolist(),
            "v": (np.array(trace["v"]) * 3.6).tolist(),  # km/h
        },
        "tbc": {"s": tbc[0].tolist(), "v": (np.array(tbc[1]) * 3.6).tolist()},
    }

    return result
