import React from 'react';
import { useNavigate } from 'react-router-dom'

export default function NavigationBar() {
    const navigate = useNavigate();

    return (
        <div style={styles.container}>
            <button style={styles.button} onClick={()=> navigate("/")}>Home</button>
            <button style={styles.button} onClick={()=> navigate("/create-game")}>Create Game</button>
            <button style={styles.button} onClick={()=> navigate("/game-overview")}>Game Overview</button>
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