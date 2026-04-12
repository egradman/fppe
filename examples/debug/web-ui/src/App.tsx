import { useState } from "react";
import "./App.css";

const HOST = window.location.hostname;
const API_PORT = 8091;
const VISER_PORT = 8090;
const API_BASE = `http://${HOST}:${API_PORT}`;

type Tab = "controls" | "teleop" | "cam0" | "cam1";

function App() {
  const [activeTab, setActiveTab] = useState<Tab>("controls");

  const tabs: { id: Tab; label: string }[] = [
    { id: "controls", label: "Controls" },
    { id: "teleop", label: "Teleop" },
    { id: "cam0", label: "Camera 0" },
    { id: "cam1", label: "Camera 1" },
  ];

  return (
    <div className="app">
      <nav className="tab-bar">
        {tabs.map((tab) => (
          <button
            key={tab.id}
            className={`tab ${activeTab === tab.id ? "active" : ""}`}
            onClick={() => setActiveTab(tab.id)}
          >
            {tab.label}
          </button>
        ))}
      </nav>
      <div className="tab-content">
        {activeTab === "controls" && (
          <iframe
            src={`http://${HOST}:${VISER_PORT}`}
            className="full-frame"
            title="Viser Controls"
            allow="autoplay; fullscreen; webgl"
          />
        )}
        {activeTab === "teleop" && (
          <div className="placeholder">Teleop</div>
        )}
        {activeTab === "cam0" && (
          <img
            src={`${API_BASE}/mjpeg/cam0`}
            className="full-frame camera-feed"
            alt="Camera 0"
          />
        )}
        {activeTab === "cam1" && (
          <img
            src={`${API_BASE}/mjpeg/cam1`}
            className="full-frame camera-feed"
            alt="Camera 1"
          />
        )}
      </div>
    </div>
  );
}

export default App;
