# Store publishing guide (Chrome Web Store / Edge Add-ons)

How to publish this extension ("nyan Real Audio Wall connector") to the Chrome
Web Store and Microsoft Edge Add-ons. Japanese version: [PUBLISHING.md](PUBLISHING.md).

## 1. Build the upload zip

From the repository root:

```powershell
.\package-chrome-extension.ps1
# -> dist\chrome-extension-<version>.zip
```

`manifest.json` ends up at the **zip root** (a store requirement); docs such as
`README.md` and `PUBLISHING*.md` are excluded. The version is read from
`manifest.json` (`version`, currently 0.3.0).

## 2. Required assets

| Purpose | Size / format | Required |
| --- | --- | --- |
| Store icon (listing) | **128 x 128 PNG** | Yes |
| Screenshots | 1280 x 800 or 640 x 400 PNG/JPEG (>= 1) | Yes |
| Small promo tile | 440 x 280 PNG | Optional |
| Marquee promo | 1400 x 560 PNG | Optional |
| In-extension icons (`icons` in `manifest.json`) | **one 128 PNG is enough** (optional) | Optional |

- The store icon is **128 x 128** - larger than the tray icon (16-32px). Export
  the same cat + AR glasses design at 128px.
- **In-extension icons are not required.** The extension loads and can be
  published with no `icons` declared; the listing then uses the 128px icon you
  upload in the dashboard.
- If you add them, **one 128px PNG is enough** (Chrome auto-resizes for other
  sizes). This extension has no toolbar button (action), so in practice only 128
  (install / store) and optionally 48 (`chrome://extensions` management page)
  matter. Declaring several sizes is an optional quality choice (no blur in each
  UI):

  ```json
  "icons": { "128": "icons/128.png" }
  // add "48": "icons/48.png", "16": "icons/16.png" only if you want crisper UIs
  ```

## 3. Publish to the Chrome Web Store

1. Open the [Developer Dashboard](https://chrome.google.com/webstore/devconsole)
   (a one-time $5 developer registration is required the first time)
2. **New item** -> upload the zip from step 1
3. **Store listing**: name and description (taken from `_locales` `ext_name` /
   `ext_desc`), category (e.g. "Tools"), languages (en / ja), icon, screenshots
4. **Privacy** tab: paste the permission justifications below, and set the data
   usage + privacy policy URL (sections 5-6)
5. Choose visibility (public / unlisted) and **submit for review**

> The listing name/description use neutral wording that fits both the OBS plugin
> and the standalone Spatial Wall app (`_locales/*/messages.json`). If you change
> the text, rebuild the zip before uploading (the extension name is a brand kept
> identical across locales).

## 4. Microsoft Edge Add-ons (optional)

The same zip can be uploaded to
[Partner Center](https://partner.microsoft.com/dashboard/microsoftedge) (it is a
Chromium extension; registration is free). Listing info and screenshots are
provided separately.

## 5. Permission justifications (ready to paste)

- **host_permissions `<all_urls>` / content script (all URLs)**: to hook
  `<audio>/<video>` and capture audio on any site the user wants to spatialize;
  the target sites cannot be known in advance.
- **`tabs`**: to identify and label each multiplexed audio stream by tab (tab
  title).
- **`system.display`**: to map a window's on-screen position to the monitor
  layout and derive the spatial direction.
- **`storage`**: to save user settings (the connection port).
- **Remote code**: none (only the bundled scripts).

## 6. Privacy / data handling

- Audio PCM and window-position metadata are sent **only to the local
  `ws://127.0.0.1:<port>`**. Nothing is sent to external servers; there is no
  analytics or tracking.
- The only stored value is the connection port (`chrome.storage.sync`).
- The Chrome Web Store requires a privacy policy URL for permissions such as
  `<all_urls>`. A short page stating "collects no personal data; audio is sent
  only to an app on the user's own PC (localhost)" is sufficient.

## 7. Updating (new version)

1. Bump `version` in `manifest.json` (e.g. 0.3.0 -> 0.3.1). See this repo's
   CONTRIBUTING for the WS protocol compatibility rules.
2. Re-run `.\package-chrome-extension.ps1` to rebuild the zip
3. Upload the new zip to the existing item in the dashboard and resubmit

> The source of truth for the extension is this `tools/chrome-extension/`. The
> standalone app ships a bundled copy, but always publish to the store from this
> source (editing the copy causes drift).
