import React from 'react';
import RobotFeed from "../components/robot_feed"
import RobotInfo from "../components/robot_info"
import Header from "../components/header"


export default function MenuPage() {
  return (
    <div style={styles.container}>
      <Header/>
      <div style={styles.content}>
        <RobotInfo/>
        <RobotFeed/>
      </div>
    </div>
  );
}

const styles = {
  container: {
    width: "100%",
    height: "100%",
  },
  content: {
    height: "50%",
    margin: "20px",
    display: "flex",
    justifyContent: "space-evenly",
  },
}