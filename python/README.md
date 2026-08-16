# Python port and simulation GUI

C++ ライブラリの Python 移植と、それを使った GUI シミュレータです。C++ のビルドは
不要で、NumPy と matplotlib、tkinter だけで動きます。

```bash
cd python
pip install -r requirements.txt
python run_gui.py
```

- モデル本体は [`vehicle_models_py/`](vehicle_models_py) — `include/vehicle_models/*.hpp`
  と 1 対 1 対応
- GUI は [`vehicle_models_py/gui/`](vehicle_models_py/gui) — 車両パラメータを 1 つ共有し、
  操縦シミュレーション／アニメーション／ハンドリング解析／性能／タイヤ／アッカーマンの 6 タブ
- 移植の妥当性検証は `python tests/test_port.py`（C++ 単体テストと同じ検査 141 項目）

ドキュメントは日本語版が [`docs_ja/`](../docs_ja)、英語版が
[`docs_en/`](../docs_en) にあります（数式は LaTeX 記法）。

| 文書 | 内容 | 日本語 | English |
|---|---|---|---|
| python-gui.md | GUI の使い方、各タブの読み方、実験例 | [ja](../docs_ja/python-gui.md) | [en](../docs_en/python-gui.md) |
| python-api.md | Python API リファレンスと C++ との差異 | [ja](../docs_ja/python-api.md) | [en](../docs_en/python-api.md) |
| validation.md | 移植の検証方法と結果 | [ja](../docs_ja/validation.md) | [en](../docs_en/validation.md) |
| models.md | 運動方程式 | [ja](../docs_ja/models.md) | [en](../docs_en/models.md) |

GUI を使わないデモ（tkinter 不要）:

```bash
python demo_animation.py                    # 単一マヌーバのアニメーション
python demo_route.py                        # data/reference_route.csv を全モデルで走行
python demo_route.py --models all --save route.gif --overview route.png
```

`demo_route.py` は 1 km の基準ルートを、同一のドライバモデル（ルートに対する
Pure Pursuit ＋ 曲率から作った速度プロファイル）で各モデルに走らせ、横偏差・速度・
横加速度を並べて見せます。詳細は
[docs_ja/python-gui.md#ルート追従](../docs_ja/python-gui.md#ルート追従)。

補助ツール:

```bash
python tools/make_doc_figures.py            # docs_en/images/*.png を再生成
python tools/compare_with_cpp.py ../build/step_steer   # C++ 実行例と数値比較
```
