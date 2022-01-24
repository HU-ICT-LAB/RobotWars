import React from 'react'

export default function SelectBox({ items }) {
  return (
    <select style={style}>
      {items?.map((item) => {
        return <option value={item.name}>{item.name}</option>;
      })}
    </select>
  );
}

const style = {
    width: "200px",
    height: "20px",
    borderRadius: "10px",
}