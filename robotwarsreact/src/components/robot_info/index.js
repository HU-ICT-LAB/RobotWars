import React from 'react';
import SelectRobotBox from "../select_robot_box"

export default function RobotInfo() {
    return(
        <div style={styles.robotInfo}>
            <SelectRobotBox/>
            <div style={styles.fields}>
                <text style={styles.field}>Name:</text>
                <text style={styles.field}>HP:</text>
                <text style={styles.field}>Description:</text>
                <text style={styles.field}>Location:</text>
            </div>
        </div>
    )
}

const styles = {
    robotInfo: {
        padding: "10px",
        backgroundColor: "#323232",
        height: "400px",
        maxWidth: "40%",
        minWidth: "500px",
        display: "flex",
        flexDirection: "column",
      },
    fields: {
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start"
    },
    field: {
        color: "white",
        marginTop: "5px"
    }
}