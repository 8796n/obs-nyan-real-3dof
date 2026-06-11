"use strict";

const portInput = document.getElementById("port");
const saved = document.getElementById("saved");

chrome.storage.sync.get({ port: 8796 }).then(({ port }) => {
  portInput.value = port;
});

document.getElementById("save").addEventListener("click", async () => {
  const port = Math.max(1024, Math.min(65535, Number(portInput.value) || 8796));
  portInput.value = port;
  await chrome.storage.sync.set({ port });
  saved.style.visibility = "visible";
  setTimeout(() => (saved.style.visibility = "hidden"), 1500);
});
