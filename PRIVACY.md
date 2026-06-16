# Privacy Policy — nyan Real Audio Wall connector

**Last updated: 2026-06-16**

This is the privacy policy for the **nyan Real Audio Wall connector** browser
extension (the Chrome/Edge extension under `tools/chrome-extension` of this
repository). It does not cover the OBS plugin or the standalone app themselves.

---

## English

### Overview

nyan Real Audio Wall connector is a browser extension that captures the audio of
playing tabs and sends it, together with each window's on-screen position, to the
nyan Real app (the OBS plugin or the standalone Spatial Wall app) running on the
**same computer**, so each browser window is heard from its position on the
virtual screen.

**Summary: the extension collects no personal data and sends nothing to external
servers. Audio and window position are sent only to an app on your own computer
over a local connection (`ws://127.0.0.1`).**

### Audio capture

The extension hooks the `<audio>/<video>` elements of pages with Web Audio to
capture their PCM audio for spatialization.

- Audio is sent **only** to the local nyan Real app at `ws://127.0.0.1:<port>`
  (default 8796) on the same machine.
- Audio is **never transmitted to any external server, cloud service, or third
  party**, and is not stored anywhere.
- No camera or microphone is used.

### Local connection only

The extension connects only to `127.0.0.1` (localhost). It opens no remote
network connection and is not reachable from outside your machine.

### Data stored locally

The extension stores a single setting using Chrome's built-in storage. No
external database or server is involved.

| Storage | What is stored |
|---|---|
| `chrome.storage.sync` | The WebSocket port number |

**`chrome.storage.sync` note:** if you are signed into Chrome with Chrome Sync
enabled, Chrome itself may sync this setting across your devices through Google's
infrastructure. This is handled entirely by Chrome and Google; the extension does
not initiate or control it. See
[Google's Privacy Policy](https://policies.google.com/privacy).

### Tab information

The extension reads tab titles (via the `tabs` permission) only to label each
audio stream in the multiplexed local connection. Tab titles and URLs are
**never transmitted outside your computer**.

### Display information

The extension reads the monitor layout (via the `system.display` permission) only
to map a window's on-screen position to a spatial direction. This information
stays local.

### Host permission (all URLs)

The content script runs on any page because audio can play on any website. It
only hooks media elements to capture their audio for spatialization and **does
not collect or transmit any personal information**.

### No analytics or tracking

The extension does **not** include analytics, telemetry, crash reporting,
advertising, tracking pixels, or any third-party SDK that collects data.

### Third-party libraries

The extension bundles no third-party libraries or SDKs. It uses only standard
browser APIs (Web Audio, WebSocket, and the `chrome.*` extension APIs).

### Changes to this policy

If this policy is updated, the "Last updated" date above will be revised.
Continued use of the extension after changes constitutes acceptance of the
updated policy.

### Contact

For questions or concerns about this privacy policy, please open an issue at:
https://github.com/8796n/obs-nyan-real-3dof/issues

---

## 日本語

### 概要

nyan Real Audio Wall connector は、再生中のタブの音声を取り込み、各ウィンドウの
画面上の位置とあわせて、**同じコンピューター上**で動作する nyan Real アプリ（OBS
プラグイン版または単独版 Spatial Wall）へ送信するブラウザ拡張機能です。これにより、
ブラウザの各ウィンドウが仮想スクリーン上の位置から聞こえるようになります。

**要約: 本拡張は個人データを収集せず、外部サーバーへ一切送信しません。音声と
ウィンドウ位置は、ローカル接続（`ws://127.0.0.1`）でお使いのコンピューター上の
アプリにのみ送信されます。**

### 音声の取り込み

本拡張は、ページの `<audio>/<video>` 要素を Web Audio でフックし、空間化のために
PCM 音声を取り込みます。

- 音声は同一マシン上のローカル nyan Real アプリ `ws://127.0.0.1:<port>`（既定
  8796）へ**のみ**送信されます。
- 音声は**いかなる外部サーバー・クラウドサービス・第三者にも送信されず**、どこにも
  保存されません。
- カメラ・マイクは使用しません。

### ローカル接続のみ

本拡張は `127.0.0.1`（localhost）にのみ接続します。リモートへのネットワーク接続は
行わず、マシンの外部から到達できるサーバーにもなりません。

### ローカルに保存されるデータ

本拡張は Chrome の組み込みストレージを使って 1 つの設定のみを保存します。外部
データベースやサーバーは一切使用しません。

| ストレージ | 保存内容 |
|---|---|
| `chrome.storage.sync` | WebSocket ポート番号 |

**`chrome.storage.sync` について:** Chrome にサインインし Chrome 同期が有効な場合、
Chrome 自体がこの設定を Google のインフラを通じて複数デバイス間で同期することが
あります。これは Chrome および Google が行うもので、本拡張が主体的に行うもの
ではありません。詳細は
[Google のプライバシーポリシー](https://policies.google.com/privacy)をご確認
ください。

### タブ情報

本拡張は（`tabs` 権限で）タブのタイトルを読み取りますが、これは多重化したローカル
接続で各音声ストリームを識別・ラベル表示するためのみに使用します。タブのタイトルや
URL はお使いのコンピューターの**外部へ送信されません**。

### ディスプレイ情報

本拡張は（`system.display` 権限で）モニタ配置を読み取りますが、これはウィンドウの
画面上の位置を空間定位の方向にマップするためのみに使用します。この情報はローカルで
完結します。

### ホスト権限（全 URL）

音声はあらゆるウェブサイトで再生されうるため、コンテンツスクリプトは全ページで
動作します。空間化のためにメディア要素の音声を取り込むだけで、**個人情報を収集・
送信することはありません**。

### アナリティクス・トラッキングなし

本拡張には、アナリティクス、テレメトリ、クラッシュレポート、広告、トラッキング
ピクセル、データを収集するサードパーティ SDK は一切含まれません。

### サードパーティライブラリ

本拡張はサードパーティのライブラリや SDK を一切同梱していません。標準のブラウザ
API（Web Audio、WebSocket、`chrome.*` 拡張 API）のみを使用しています。

### ポリシーの変更

このポリシーを更新した場合、上部の「Last updated」日付を更新します。変更後も拡張
機能を継続して使用することで、更新されたポリシーへの同意とみなします。

### お問い合わせ

このプライバシーポリシーに関するご質問・ご意見は、以下の GitHub Issues までお寄せ
ください：
https://github.com/8796n/obs-nyan-real-3dof/issues
