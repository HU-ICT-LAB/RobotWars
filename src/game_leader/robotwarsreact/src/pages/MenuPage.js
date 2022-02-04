import React from "react";
import RobotInfo from "../components/RobotInfo";
import Header from "../components/Header";

export default function MenuPage() {
  return (
    <div style={styles.container}>
      <Header />
      <div style={styles.content}>
        <RobotInfo />
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
};
