import React, { useState, useEffect } from "react";
import SelectBox from "./SelectBox";

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
        <SelectBox items={robots} onChange={handleChange}></SelectBox>
      <div style={styles.fields}>
        <text style={styles.field}>
          {"Name: " + (selectedRobot?.name === null ? "" : selectedRobot?.name)}
        </text>
        <text style={styles.field}>
          {"HP: " + (selectedRobot?.hp === null ? "" : selectedRobot?.hp)}
        </text>
        <text style={styles.field}>
          {"Description: " +
            (selectedRobot?.description === null
              ? ""
              : selectedRobot?.description)}
        </text>
        <text style={styles.field}>
          {"Location: " +
            (selectedRobot?.location === null ? "" : selectedRobot?.location)}
        </text>
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
    width: "55%",
    display: "flex",
    flexDirection: "column",
    borderRadius: "10px",
  },
  title: {
    color: "white",
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
};
