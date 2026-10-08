<img width="4080" height="3060" alt="IMG_20261008_153851989_HDR" src="https://github.com/user-attachments/assets/43a361eb-259c-46a7-be2e-6ab870c12d6a" />

# PSP LCDC Capture

PSPのLCDC映像と音声をUSB経由でPCへ表示と再生するARK-5用PRX。

## 使い方

1. `lcdc_capture.prx` をPSPへ入れ、ARK-5のプラグインとして有効化
2. PSPをUSBでPCへ接続
3. PCで `pc/PSP_LCDC_Capture_Start.bat` を起動
4. OBSのブラウザソースで `pc/obs.html` を**ローカルファイル**として指定（480×272）
5. ゲームを起動すると自動でキャプチャ開始

## PS1 / POPS

PS1（POPS）では、PSP本体に最初から入っているUSBモジュール `flash0:/kd/usb.prx` も有効化してください。別途ダウンロードやコピーは不要です。

ARK-5の `PLUGINS.TXT` では、`usb.prx` を先に読み込むようにします。

```text
pops, flash0:/kd/usb.prx, on
pops, ms0:/seplugins/lcdc_capture.prx, on
```

`lcdc_capture.prx` の配置先が違う場合は、2行目のパスだけ合わせてください。

## 操作入力

このPRXは**映像と音声のキャプチャ専用**です。操作入力はPSP本体のボタンを使うか、RemoteJoyLiteなど別のRemoteJoy系ツールの**入力機能のみ**を利用してください。

## ビルド

PSPSDKを設定したWSL / Ubuntuで `build.sh` を実行してください。

ビルド後、`dist/lcdc_capture.prx` と配布用 `dist/PSP-LCDC-Capture.zip` が生成されます。
