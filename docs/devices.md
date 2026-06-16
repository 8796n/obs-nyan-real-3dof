# 対応機種一覧

nyan Real 3DoF / Spatial Wall が認識する AR グラスの一覧と、機種ごとの解像度・
対角 FoV・専用機能・実機確認状況。

> 出典: 機種データはすべて [`src/core/device_registry.cpp`](../src/core/device_registry.cpp)
> が単一のソースです(プロファイル= `model_profile`、トランスポート特性=
> `transport_traits`)。本ドキュメントはそこからの抜き書きなので、機種を追加・
> 変更したらこの表も更新してください。

## 認識のしくみ

- **IMU(姿勢)とプロファイルの選択**: USB HID の VID:PID(必要に応じて
  ProductString の部分一致)で機種を判別し、トランスポート/FoV/解像度/マウント
  角などの `model_profile` を選びます。最初に一致した行が採用され、`devices.json`
  のユーザー定義が組み込みより優先されます。
- **メガネ画面(ディスプレイ)の判別**: ワープ出力先や自動除外のため、ディスプレイ
  パネルを EDID(ベンダー PNP ID + 製品コード/モニタ名)で識別します(末尾の表)。
- **専用機能の出し分け**: ドックの行はトランスポート特性とプロファイルのフラグで
  自動的に出し分けられます(機種をハードコードしていません)。

## 機種一覧

実機確認: ✅=確認済み / ❌=未確認(未入手等) / ⚠=未記録(要確認)。
**この列はコード注記と開発メモからの暫定値です。実機で確認したら更新してください。**

| 機種 | USB VID:PID | トランスポート | 解像度 | 対角FoV | 専用機能 | 実機確認 |
|---|---|---|---|---|---|---|
| XREAL One Pro | 3318:0435 / 0436 | One bridge (TCP) | 1920×1080 | 57° | Eyeカメラ, ネットワーク接続 | ✅ |
| XREAL One | 3318:0437 / 0438 | One bridge (TCP) | 1920×1080 | 50° | Eyeカメラ, ネットワーク接続 | ✅ |
| XREAL 1S | 3318:043D / 043E | One bridge (TCP) | 1920×1080 | 52° | Eyeカメラ, ネットワーク接続 | ✅ |
| ROG XREAL R1 | 0B05:1D9C / 1D9D | One bridge (TCP) | 1920×1080 | 57° | Eyeカメラ, ネットワーク接続 | ⚠ |
| XREAL Air / Air 2 | 3318:0424 / 0432 / 0428 / `*Air*` | air_hid | 1920×1080 | 46° | 表示モード切替 | ✅ |
| RayNeo Air | 1BBB:AF50 / `*RayNeo*` | rayneo_hid | 1920×1080 | 46° | — | ✅† |
| EPSON MOVERIO BT-40 | 04B8:0D12 | Windows Sensor API | 1920×1080 | 34° | 表示モード切替(2D/3D), 輝度調節, 自動調光, 輻輳連動(光学焦点4.6m) | ❌ |
| EPSON MOVERIO BT-30C | 04B8:0C0C | Windows Sensor API | 1280×720 | 23° | 表示モード切替(2D/3D), 輝度調節 | ✅ |
| Rokid Max | 04D2:162F `*Max*` / `*Rokid*` | rokid_hid | 1920×1080 | 48.8°※ | — | ✅ |
| Rokid Air | 04D2:162F | rokid_hid | 1920×1080 | 43° | — | ✅ |
| VITURE One | 35CA:1011 / 1013 / 1017 / `*VITURE*` | viture_hid | 1920×1080 | 43° | 表示モード切替 | ✅ |
| VITURE One Lite | 35CA:1015 / 101B | viture_hid | 1920×1080 | 43° | 表示モード切替 | ❌ |
| VITURE Pro | 35CA:1019 / 101D | viture_hid | 1920×1080 | 46° | 表示モード切替 | ❌ |
| Nreal Light | 05A9:0680 `*OV580*` + 0486:573C | nreal_hid | 1920×1080 | 52° | 表示モード切替 | ✅ |

`*Air*` などは「その VID の他の PID でも ProductString に該当文字列を含めば同プロ
ファイル」のワイルドカード行です。XREAL Ultra は意図的に未対応(unknown 扱い)。

※ Rokid Max の 48.8° は 1920×1080 出力時のレターボックス込みの実効値です(ネイティブ
1920×1200 パネルの公称は 50°)。

† RayNeo は **RayNeo Air 4 Pro** で動作確認済み。RayNeo 系は単一の "RayNeo Air"
プロファイル(46° / 1920×1080)で扱っているため、他の RayNeo 機種が同プロファイルで
そのまま動くか(FoV やマウント角が合うか)は未確認です。

## 専用機能の詳細

- **表示モード切替**(`displaymode` 行): トランスポートごとに選択肢が異なります。
  - XREAL Air 系(air_hid): ミラー 60 / 72 / 90 / 120Hz、SBS 60 / 72Hz。
    120Hz は Air 2 パネルが必要。Half-SBS(8)と SBS 90Hz(9)は SET は通るが画面が
    黒くなるため除外(Air 2 Pro / Air 2 で実機確認済み)。
  - VITURE(viture_hid): 2D 1920×1080 / 3D SBS 3840×1080(リフレッシュ切替は無し)。
  - Nreal Light(nreal_hid): ミラー 60 / SBS 60 / SBS 72Hz(SBS 72 は ELLA リビジョン
    が必要)。
  - MOVERIO(sensor_api): 2D / 3D(Half-SBS)。Basic Function SDK の `set2d3d`
    コマンド。BT-40 / BT-30C 両対応。**3D を選ぶと「SBS出力」が自動でも有効**に
    なります(Half-SBS=1920×1080 は解像度から判別できないため、3D 表示モードを
    根拠に判定)。明示的に「SBS出力=オフ」にしている場合のみ無効のまま。
  - 上記以外(One 系 / RayNeo / Rokid)は表示モード切替なし。
- **輝度調節**(`brightness` 行): MOVERIO **BT-40 / BT-30C** 両方。ハードの明るさを
  シリアルコマンドポート経由で調整(機種プロファイルの `display_brightness`)。
- **自動調光**(`autobright` 行): MOVERIO **BT-40 のみ**(BT-30C は手動輝度のみで
  自動モード非対応。プロファイルの `display_autobright` で出し分け)。
- **輻輳連動**(`convergence_link` 行): MOVERIO の `setdisplaydistance`(左右像の水平
  シフトで知覚距離を動かす)に対応する機種のみ=**BT-40**。BT-30C / BT-35E / BT-30E は
  非対応(SDK 付録準拠)。0 シフト時の知覚距離 4.6m を光学焦点としても採用。
- **Eyeカメラ(UVC トグル)**(`dock.eye` 行): XREAL One ファミリー(One bridge/TCP)
  の Eye カメラアクセサリの着脱状態と UVC 有効/無効。
- **ネットワーク接続**(IP / ポート): XREAL One ファミリー(One bridge は TCP 接続)。

## メガネ画面パネルの EDID 識別子

ワープ出力先の特定とウォールからの自動除外に使用します。

| 機種系統 | ベンダー(PNP) | 製品コード / モニタ名 |
|---|---|---|
| XREAL(全世代) | MRG | ベンダーのみで一致(Air 0x3132 / Air 2 0x3134 / One Pro 0x4100 / 1S 0x4102) |
| RayNeo | TCL | 製品 0x03D4 または名前 "SmartGlasses"(TCL は一般モニタと共有のため限定) |
| EPSON MOVERIO BT-40 | SEC | 製品 0xD004 |
| EPSON MOVERIO BT-30C | SEC | 製品 0xD003 |
| Rokid | LBT | 名前 "Rokid"(Rokid Max = 0x4753) |
| VITURE | CVT | 名前 "VITURE"(汎用 CVT ベンダーのため名前で限定) |
| Nreal Light | NRL | ベンダーのみ(旧世代で MRG 以前) |

## 未対応機種の追加(devices.json)

組み込み表に無い機種は、再ビルドせずに `devices.json` で追加できます(プラグイン
設定ディレクトリ。書式は [`data/devices-example.json`](../data/devices-example.json))。
最低限 `vid` / `pid` / `transport` / `mount` が必要で、`fov_deg` / `display_width` /
`display_height` / `optics_focus_m` / `display_distance` / `product_contains` /
EDID 各項目は任意です。ユーザー定義は組み込みより優先されるので、既存プロファイルの
上書きにも使えます。
