import React from "react";
import NavigationBar from "./NavigationBar";
import logo192 from "../assets/logo192.png";

export default function Header() {
  return (
    <div style={styles.header}>
      <img src={logo192} style={styles.logo} alt="Logo of robomaster"></img>
      <h1 style={styles.title}>RobotWars</h1>
      <NavigationBar></NavigationBar>
    </div>
  );
}

const styles = {
  title: {
    color: "#14FFEC",
    alignSelf: "center",
  },
  logo: {
    marginTop: "10px",
    width: "96px",
    height: "96px",
    alignSelf: "center",
  },
  header: {
    display: "flex",
    flexDirection: "column",
  },
};
