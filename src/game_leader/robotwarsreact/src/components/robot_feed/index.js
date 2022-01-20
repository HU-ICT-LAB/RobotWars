import React, { useState } from "react";

export default function RobotFeed() {
  let [ip, setIp] = useState(null);
  let ipRef = React.createRef();

  function handleChange() {
    setIp(ipRef.current.value)
    console.log(ip)
  }

  return (
    <div style={styles.robotFeed}>
      <h2 style={styles.text}>Live game camera</h2>
      <input ref={ipRef} placeholder="IP of the camera"/>
      <button onClick={handleChange}>Update IP</button>
      <iframe
        style={styles.cam}
        src={ip}
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
    marginTop: "10px",
    height: "396px",
    width: "704px",
    borderRadius: "10px",
  },
};
