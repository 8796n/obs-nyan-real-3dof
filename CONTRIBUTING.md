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

## 開発フロー

`main` への変更は、次の流れを原則とします（`main` への直接 push はしません）。

1. 変更内容を説明する issue を立てる
2. トピックブランチで PR を作成し、issue を参照する
3. レビューを経て `main` へマージする（squash マージ）

リリースは `main` 上のコミットへ `v*` タグを push すると、GitHub Actions
（`.github/workflows/release.yml`）がビルドして ZIP とインストーラーを
Releases へ公開します。
