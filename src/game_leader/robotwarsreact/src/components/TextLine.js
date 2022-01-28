import React from "react";

export default function TextLine( {type, value} ) {
  return(
    <text style={style}>{type + ": " + value}</text>
  );
}

const style = {
  color: "white",
  marginTop: "5px",
}