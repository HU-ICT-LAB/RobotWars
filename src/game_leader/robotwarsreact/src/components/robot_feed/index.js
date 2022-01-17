import React from 'react';

export default function RobotFeed() {
    return(
        <div style={styles.robotFeed}>
            <iframe style={styles.cam} src="http://192.168.178.11:4747/video" allow="camera;"></iframe>
        </div>
    )
}

const styles = {
    robotFeed: {
        padding: "10px",
        backgroundColor: "#323232",
        height: "360px",
        width: "640px",
        borderRadius: "10px"
      },
      cam: {
          width: "100%",
          height: "100%"
      }
}