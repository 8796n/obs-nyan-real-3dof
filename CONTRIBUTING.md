# コントリビューションガイド

## 言語方針

本プロジェクトでは、用途に応じて言語を使い分けます。

### 内部資料・開発のやりとり → 日本語が原則

- コミットメッセージ、Pull Request の説明、Issue
- 開発者向けドキュメント（`docs/` 配下、例: `docs/architecture.md`）
- 開発上の議論・レビューコメント

### 配布物（プラグイン本体）→ 英語をデフォルト、日本語版も用意

エンドユーザーの目に触れるものは英語をデフォルトとし、日本語版を併設します。
ただし README は例外として日本語をメイン（`README.md`）とし、英語版
（`README.en.md`）を併設します。

| 対象 | 英語 | 日本語 |
|---|---|---|
| README | `README.en.md` | `README.md`（メイン） |
| UI ロケール | `data/locale/en-US.ini`（デフォルト） | `data/locale/ja-JP.ini` |
| インストーラ文言 | `installer/obs-nyan-real-3dof.iss` の `en.*`（デフォルト） | 同 `ja.*` |
| ソースコードのコメント・ログ・`obs_module_description` | 英語 | （不要） |

補足:

- OBS のデフォルトロケールは英語に設定しています
  (`OBS_MODULE_USE_DEFAULT_LOCALE("obs-nyan-real-3dof", "en-US")`)。
- ロケールキーを追加・変更した場合は、`en-US.ini` と `ja-JP.ini` の両方を必ず
  更新してください。
- README を更新した場合は、`README.md`（日本語・メイン）と `README.en.md`（英語）の
  両方を同期させてください。冒頭の言語切り替えリンクも維持します。
- `docs/` 配下は開発者向けの内部資料のため、配布パッケージ（`package.ps1`）には
  同梱しません。例外は README が参照するスクリーンショット（`docs/images/`）で、
  これだけは配布物にも含めます。

## Audio Wall ⇔ Chrome 拡張の WebSocket プロトコル

プラグイン（`src/audio-wall-source.cpp`）と Chrome 拡張
（`tools/chrome-extension/`）は localhost WebSocket（既定 8796）で会話します。
拡張はストア配布だと自動更新、プラグインは手動更新のため、版ズレは
いつか必ず起きます。黙って壊れないために、次のルールを守ってください。

- プロトコルバージョンは 2 箇所で定義されています。**変更時は必ず両方を
  同時に更新**します:
  - プラグイン側: `src/audio-wall-source.cpp` の `WS_PROTOCOL_VERSION`
  - 拡張側: `tools/chrome-extension/background.js` の `PROTOCOL_VERSION`
    （meta メッセージの `"v"` フィールドとして送信される）
- バージョンを +1 するのは**互換性が壊れる変更**のときだけです:
  メッセージ形式やバイナリフレームのレイアウト変更、既存フィールドの
  意味変更・削除など。**フィールドの追加だけなら上げません**
  （受信側は未知フィールドを無視する規約）。
- バージョン不一致のとき、プラグインは接続ごとに 1 回だけ
  `extension speaks ingest protocol vX but this plugin expects vY` を
  警告ログに出し、処理はベストエフォートで続けます。プロトコルを
  変更したら、この警告が新旧の組み合わせで実際に出ることを確認して
  ください。
- 拡張の `manifest.json` の `version` はストアへ出すたびに上げますが、
  プロトコルバージョンとは独立です（プロトコルが同じならいくつ上げても
  よい）。

## 開発フロー

`main` への変更は、次の流れを原則とします（`main` への直接 push はしません）。

1. 変更内容を説明する issue を立てる
2. トピックブランチで PR を作成し、issue を参照する
3. レビューを経て `main` へマージする（squash マージ）

リリースは `main` 上のコミットへ `v*` タグを push すると、GitHub Actions
（`.github/workflows/release.yml`）がビルドして ZIP とインストーラーを
Releases へ公開します。
