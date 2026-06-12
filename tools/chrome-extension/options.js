"use strict";

// Visible strings come from _locales/ so the page follows the browser's UI
// language (the extension name stays a brand and is identical everywhere).
const msg = (key) => chrome.i18n.getMessage(key);
document.documentElement.lang = chrome.i18n.getUILanguage();
document.title = msg("ext_name");
document.getElementById("heading").textContent = msg("ext_name");
document.getElementById("port-label").textContent = msg("options_port_label");
document.getElementById("save").textContent = msg("options_save");
document.getElementById("note").textContent = msg("options_note");

const portInput = document.getElementById("port");
const saved = document.getElementById("saved");
saved.textContent = msg("options_saved");

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
