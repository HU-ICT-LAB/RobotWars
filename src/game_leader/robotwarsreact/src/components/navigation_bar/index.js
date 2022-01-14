import React from 'react';

export default function NavigationBar() {
    return (
        <div style={styles.container}>
            <button style={styles.button} onClick={()=> window.location = "/"}>Home</button>
            <button style={styles.button} onClick={()=> window.location = "/create-game"}>Create Game</button>
            <button style={styles.button} onClick={()=> window.location = "/game-overview"}>Game Overview</button>
        </div>
        );
}

const styles = {
    container: {
        width: "80%",
        backgroundColor: "#323232",
        alignSelf: "center",
        display: "flex",
        justifyContent: "space-evenly",
        padding: "5px",
        borderRadius: "10px"
    },
    button: {
        width: "150px",
        height: "30px",
        backgroundColor: "#0D7377",
        borderRadius: "10px",
        borderStyle: "hidden",
        color: "white"
    }
}