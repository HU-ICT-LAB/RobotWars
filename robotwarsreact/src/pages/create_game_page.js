import React from 'react';
import Header from "../components/header"

export default function CreateGamePage() {
  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <Header/>
      </div>
      <div style={styles.content}>
        <div>
          Create game
          field
          field
          field
        </div>
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