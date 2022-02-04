import Header from "../components/Header";
import RobotFeed from "../components/RobotFeed";
import GameOverview from "../components/GameOverview";

export default function GameOverviewPage() {
  return (
    <div style={styles.container}>
      <Header />
      <div style={styles.content}>
        <GameOverview />
        <RobotFeed />
      </div>
    </div>
  );
}

const styles = {
  container: {
    width: "100%",
    height: "100%",
  },
  title: {
    color: "white",
  },
  content: {
    height: "50%",
    margin: "20px",
    display: "flex",
    justifyContent: "space-evenly",
  },
};
