import React from 'react';
import "./menu_button.css"

export default function MenuButton({text, adress}) {

  function action() {
    fetch('/api/v1/' + adress)
    .then(response => {
      if (response.ok) {
      console.log(response.json())
      }
    })
  }
  
  return (
  <button className="button" onClick={action}>{text}</button>
  );
}