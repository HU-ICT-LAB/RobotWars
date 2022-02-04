import './App.css';
import MenuPage from "./pages/MenuPage"
import CreateGamePage from "./pages/CreateGamePage"
import GameOverviewPage from "./pages/GameOverviewPage"
import { Routes, Route } from "react-router-dom"

export default function App() {
  return (
    <div className="App">
      <Routes>
        <Route path="/" element={<MenuPage/>}></Route>
        <Route path="create-game" element={<CreateGamePage/>}></Route>
        <Route path="game-overview" element={<GameOverviewPage/>}></Route>
      </Routes>
    </div>
  );
}
