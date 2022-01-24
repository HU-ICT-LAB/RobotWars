import React from "react";
import Header from "../components/header";
import CreateGame from "../components/create_game";

export default function CreateGamePage() {
  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <Header />
      </div>
      <div style={styles.content}>
        <CreateGame />
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
