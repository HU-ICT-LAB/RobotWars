import React, { useState, useEffect } from "react";

import TextLine from "./TextLine";

export default function RobotInfo() {
  const [robots, setRobots] = useState([]);
  let [selectedRobot, setSelectedRobot] = useState(null);

  useEffect(() => {
    fetch("http://127.0.0.1:5000/api/v1/robots")
      .then((response) => response.json())
      .then(
        (data) => {
          setRobots(data[0]);
          setSelectedRobot(data[0][0]);
        },
        (error) => {
          window.alert(error);
        }
      );
  }, []);

  function handleChange(event) {
    setSelectedRobot(robots.find((robot) => robot.name === event.target.value));
  }
  return (
    <div style={styles.robotInfo}>
      <h2 style={styles.title}>Robot Information</h2>
      <div style={styles.fields}>

      </div>
    </div>
  );
}

const styles = {
  robotInfo: {
    paddingLeft: "30px",
    paddingRight: "30px",
    backgroundColor: "#323232",
    height: "500px",
    maxWidth: "40%",
    minWidth: "500px",
    display: "flex",
    flexDirection: "column",
    borderRadius: "10px",
  },
  title: {
    color: "white",
    display: "flex",
    alignItems: "flex-start",
  },
  select: {
    width: "200px",
    height: "20px",
    borderRadius: "10px",
  },
  fields: {
    display: "flex",
    flexDirection: "column",
    alignItems: "flex-start",
  },
  field: {
    color: "white",
    marginTop: "5px",
  },
  selecter: {
    width: "200px",
    height: "20px",
    borderRadius: "10px",
  }
};
