// frontend/src/hooks/useSimulationSocket.js
import { useEffect } from "react";
import { io } from "socket.io-client";

const BACKEND_URL = "https://fuzzy-space-spoon-q77rggjpg5rxc4wq4-8080.app.github.dev";

// 소켓을 모듈 최상단에서 한번만 생성해서 재사용
const socket = io(BACKEND_URL, {
  transports: ["websocket"],
  // 필요하면 옵션 추가
  // withCredentials: true,
  // extraHeaders: { /* 헤더 설정 가능 */ },
});

const useSimulationSocket = (onMessage) => {
  useEffect(() => {
    if (!socket.connected) {
      socket.connect();
    }

    socket.on("connect", () => {
      console.log("✅ Socket.IO 연결됨");
    });

    socket.on("state", (data) => {
      onMessage(data);
    });

    socket.on("disconnect", () => {
      console.warn("🔌 Socket.IO 연결 종료");
    });

    // 컴포넌트 언마운트 시 이벤트 리스너만 제거, socket은 닫지 않음
    return () => {
      socket.off("state", onMessage);
      socket.off("connect");
      socket.off("disconnect");
    };
  }, [onMessage]);
};

export default useSimulationSocket;
