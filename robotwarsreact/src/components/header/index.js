import React from 'react';
import NavigationBar from "../navigation_bar"
import logo192 from "../../logo192.png"

export default function Header() {
    return(
        <div style={styles.header}>
            <img src={logo192} style={styles.logo}></img>
            <h1 style={styles.title}>RobotWars</h1>
            <NavigationBar></NavigationBar>
        </div>
    )
}

const styles = {
    title: {
        color: "#14FFEC",
        paddingBottom: "20px",
        alignSelf: "center"
      },
      logo: {
        width: "96px",
        height: "96px",
        alignSelf: "center"
      },
      header: {
        display: "flex",
        flexDirection: "column"
      },
}