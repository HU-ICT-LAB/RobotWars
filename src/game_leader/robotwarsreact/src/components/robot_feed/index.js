import React from "react";

export default function RobotFeed() {
  return (
    <div style={styles.robotFeed}>
      <h2 style={styles.text}>Live game camera</h2>
      <iframe
        style={styles.cam}
        src="http://192.168.178.11:4747/video"
        allow="camera;"
      ></iframe>
    </div>
  );
}

const styles = {
  robotFeed: {
    backgroundColor: "#323232",
    height: "520px",
    width: "800px",
    borderRadius: "10px",
  },
  text: {
    color: "white",
  },
  cam: {
    height: "396px",
    width: "704px",
    borderRadius: "10px",
  },
};
