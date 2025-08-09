from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO
import threading
import time
import json
import os

app = Flask(__name__)
CORS(app, supports_credentials=True)  # CORS 인증 지원

socketio = SocketIO(app, cors_allowed_origins="*")

# 상태 초기화
state = {
    "running": False,
    "notch": 0,
    "velocity": 90.0,            # km/h
    "position": 500.0,           # m
    "L": 500.0,                  # 전체 거리 (m)
    "remaining_distance": 500.0, # 남은 거리 (m)
}

@app.before_request
def before_request_func():
    # OPTIONS 요청에 대해 200 응답 처리 (CORS 프리플라이트 대응)
    if request.method == 'OPTIONS':
        return '', 200


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SCENARIO_PATH = os.path.join(BASE_DIR, "stopping-sim/data", "scenario.json")

def load_scenario():
    print(f"Scenario 파일 경로: {SCENARIO_PATH}")
    if os.path.exists(SCENARIO_PATH):
        with open(SCENARIO_PATH, "r") as f:
            scenario_data = json.load(f)
            print(f"로드한 scenario 데이터: {scenario_data}")
            state.update(
                {
                    "L": scenario_data.get("L", 500),
                    "v0": scenario_data.get("v0", 90),
                    "grade_percent": scenario_data.get("grade_percent", 0),
                    "mu": scenario_data.get("mu", 0),
                    "dt": scenario_data.get("dt", 0.01),
                    "velocity": scenario_data.get("v0", 90),
                    "position": scenario_data.get("L", 500),
                    "remaining_distance": scenario_data.get("L", 500),
                }
            )
            
    else:
        print("scenario.json 파일을 찾을 수 없습니다.")
@app.route("/control", methods=["POST"])
def control():
    data = request.json
    action = data.get("action")
    print(f"받은 액션: {action}")

    if action == "start":
        state["running"] = True
    elif action == "stop":
        state["running"] = False
    elif action == "notch_up" and state["notch"] < 7:
        state["notch"] += 1
    elif action == "notch_down" and state["notch"] > 0:
        state["notch"] -= 1

    print(f"현재 노치: {state['notch']}")
    return jsonify(state)


# 상태 조회 API: GET /state
@app.route("/state", methods=["GET"])
def get_state():
    return jsonify(state)
def sim_loop():
    dt = 0.1
    while True:
        if state["running"]:
            decel_m_s2 = 0.1 * state["notch"]
            velocity_m_s = state["velocity"] * 1000 / 3600
            velocity_m_s = max(velocity_m_s - decel_m_s2 * dt, 0)
            state["velocity"] = velocity_m_s * 3600 / 1000
            state["position"] -= velocity_m_s * dt
            state["remaining_distance"] = max(0, state["L"] - state["position"])

            print(f"속도(km/h): {state['velocity']:.2f}, 위치(m): {state['position']:.2f}, 노치: {state['notch']}")

            if state["position"] <= 0:
                state["position"] = 0
                state["velocity"] = 0
                state["running"] = False

            socketio.emit("state", state, broadcast=True)

        time.sleep(dt)

# 시뮬레이션 쓰레드 시작 (데몬 모드)
threading.Thread(target=sim_loop, daemon=True).start()

# WebSocket 연결 시 이벤트
@socketio.on("connect")
def handle_connect():
    print("✅ 클라이언트 연결됨")
    socketio.emit("state", state)

if __name__ == "__main__":
    # Flask-SocketIO 서버 실행 (0.0.0.0:8080)
    socketio.run(app, host="0.0.0.0", port=8080)
