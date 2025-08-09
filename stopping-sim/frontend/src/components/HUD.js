// frontend/src/components/HUD.js
import React, { useState, useEffect } from "react";
import styled from "styled-components";
import { runSimulation } from "../api";
import useSimulationSocket from "../hooks/useSimulationSocket";

const HUD = () => {
  const notches = ["N", "B1", "B2", "B3", "B4", "B5", "B6", "B7"];

  const [speed, setSpeed] = useState(0);
  const [notch, setNotch] = useState(0);
  const [distance, setDistance] = useState(0);
  const [running, setRunning] = useState(false);

  useSimulationSocket((data) => {
    setSpeed(data.velocity ?? 0);
    setNotch(data.notch ?? 0);
    setDistance(data.remaining_distance ?? 0);
    setRunning(data.running ?? false);
  });

  const handleRun = async () => {
    try {
      await runSimulation("start");
    } catch (err) {
      console.error("시뮬레이션 실행 실패:", err);
    }
  };

  const handleNotchUp = async () => {
    try {
      await runSimulation("notch_up");
    } catch (err) {
      console.error("노치 업 실패:", err);
    }
  };

  const handleNotchDown = async () => {
    try {
      await runSimulation("notch_down");
    } catch (err) {
      console.error("노치 다운 실패:", err);
    }
  };

  // 키보드 제어 (↑: notch_up, ↓: notch_down)
  useEffect(() => {
    const handleKeyDown = (event) => {
      if (event.key === "ArrowUp") {
        handleNotchUp();
      } else if (event.key === "ArrowDown") {
        handleNotchDown();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, []);

  return (
    <Container>
      <Header>🚄 TASC HUD</Header>

      <Info>
        <span>속도</span>
        <span>{speed.toFixed(1)} km/h</span>
      </Info>
      <Info>
        <span>브레이크</span>
        <span>{notch === 0 ? "N" : `B${notch}`}</span>
      </Info>
      <Info>
        <span>남은 거리</span>
        <span>{distance.toFixed(1)} m</span>
      </Info>

      <Button onClick={handleRun} disabled={running}>
        {running ? "시뮬레이션 중..." : "실행"}
      </Button>
    </Container>
  );
};

export default HUD;

// --- Styled Components ---
const Container = styled.div`
  padding: 2rem;
  background-color: #1a1a1a;
  color: #fff;
  border-radius: 10px;
  width: 350px;
  display: flex;
  flex-direction: column;
  gap: 1rem;
`;

const Header = styled.h2`
  font-size: 24px;
  color: #ff6347;
`;

const Info = styled.div`
  display: flex;
  justify-content: space-between;
  font-size: 18px;
`;

const Button = styled.button`
  background-color: #28a745;
  color: white;
  border: none;
  padding: 12px;
  border-radius: 5px;
  cursor: pointer;
  font-size: 18px;

  &:hover {
    background-color: #218838;
  }

  &:disabled {
    background-color: #6c757d;
    cursor: not-allowed;
  }
`;
