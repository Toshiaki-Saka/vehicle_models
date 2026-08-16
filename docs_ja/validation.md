# Python 移植の検証

English edition: [`docs_en/validation.md`](../docs_en/validation.md)

Python 版のモデルは「C++ と*同じ*モデルである」ことが保証されて初めて意味を
持ちます。その裏付けとして、独立した3つの検査を用意しています。

---

## 1. C++ 単体テストを Python で再実行

[`python/tests/test_port.py`](../python/tests/test_port.py) は
`test/test_kinematics.cpp` / `test/test_dynamics.cpp` / `test/test_integrator.cpp`
を1対1で写したものです。車両も操縦も許容誤差もすべて同じです。

```bash
cd python
python tests/test_port.py
```

```
checks run : 141
all checks passed
```

検査項目：

| 区分 | 検査内容 |
|---|---|
| ユニサイクル | $2\pi/\omega$ 後に円が閉じること、直進距離、その場旋回 |
| 差動二輪 | 車輪角速度 ↔ 車体速度の往復変換、純回転、走行距離 |
| アッカーマン幾何 | 理想条件 $\cot\delta_o - \cot\delta_i = T/L$、左右対称性、半径・ハンドル角の往復変換、最小回転半径 |
| 車輪速 | 後輪の平均が車体速度に一致、外輪の方が速い、角速度換算 |
| キネマティック二輪 | ヨーレート $v\tan\delta/L$、円軌道が閉じること、3つの基準点の等価性、加速と上限クランプ |
| 操舵アクチュエータ | ステップ直後のレート制限、指令値への収束、機械的リミット |
| タイヤモデル | 線形の傾きと飽和、Fiala の傾き／単調性／すべり域の値、Pacejka の $BCD$ とピーク、摩擦楕円（0 / 0.6 / 1.0） |
| 線形ハンドリング解析 | 特性速度でゲインがニュートラル値の半分、$\delta = L/R + K a_y$、アンダーステア車が全速度域で安定、オーバーステア車が限界速度以上で不安定 |
| 線形モデル vs 解析解 | 20 s のシミュレーションが `steadyStateCornering()` に $10^{-6}$ 以内で一致 |
| ダイナミック二輪 | 定常値が 2 % 以内、直進が平衡点、指令加速度、荷重移動の総和が $mg$、停止時も微分が有限、$\lvert F_y \rvert \le \mu F_z$ |
| ブレンド | 両端と中間でのブレンド係数、極低速でキネマティック予測に一致、ブレンド速度以上で純ダイナミックと一致（$10^{-12}$） |
| ダブルトラック | 静荷重配分、制動時・旋回時の荷重移動、荷重総和が常に $mg$、内輪の方が大きく切れる、限界以下で単輪モデルと一致（5 %）、$a_y$ が有界、複合スリップで横力が減る |
| パラメータ検証 | 3つのプリセットが妥当、意図的に壊した設定がちょうど3件の違反を報告 |
| 積分器の次数 | 四分円軌道で Euler / Heun / RK4 の誤差比が 2 / 4 / 16 |
| runner / performance | ダイナミックモデルが解析解の定常値に到達、キネマティックが過大評価、速度コントローラが動作点を保持、制動距離と限界横加速度が物理的に妥当 |

収束次数の検査は刻み幅を半分にしたときの誤差比

$$
\frac{\lVert e(h) \rVert}{\lVert e(h/2) \rVert} \approx 2^{p},
\qquad p = 1,\ 2,\ 4
$$

（それぞれ Euler / Heun / RK4）で行っています。

---

## 2. C++ ドキュメント記載値の再現

C++ の `README.md` に記載した数値は、Python 移植でそのまま再現されます。

| 量 | C++ README | Python 移植 |
|---|---|---|
| アンダーステア勾配（乗用車） | `+0.0034 rad/(m/s²)` / `+1.917 deg/g` | `+0.0034` / `+1.917 deg/g` |
| スタティックマージン | `+0.1056` | `+0.1056` |
| 特性速度 | `28.13 m/s (101.3 km/h)` | `28.13 m/s (101.3 km/h)` |
| 20 m/s・3 deg ステップ操舵 — キネマティックのヨーレート | `0.388 rad/s` | `0.388206 rad/s` |
| 20 m/s・3 deg ステップ操舵 — 単輪のヨーレート | `0.248 rad/s` | `0.247893 rad/s` |

再現手順：

```bash
cd python
python -c "
from vehicle_models_py import make_passenger_car_parameters as car
from vehicle_models_py import linear_analysis as a
p = car()
print('%+.3f deg/g' % a.understeer_gradient_deg_per_g(p))
print('%+.4f' % a.static_margin(p))
print('%.2f m/s' % a.characteristic_speed(p))
"
```

---

## 3. コンパイル済み実行例との直接比較

[`python/tools/compare_with_cpp.py`](../python/tools/compare_with_cpp.py) は
ビルド済みの `step_steer` を実行し、同一の実験を Python 側で再現して、チャネル
ごとの最大絶対差

$$
\varepsilon_{\text{channel}}
= \max_{k}\ \bigl\lvert y^{\text{C++}}_k - y^{\text{Py}}_k \bigr\rvert
$$

を報告します。

```bash
cmake -S . -B build -DVEHICLE_MODELS_BUILD_EXAMPLES=ON
cmake --build build -j
cd python
python tools/compare_with_cpp.py ../build/step_steer
```

両者は同じ方程式・同じ RK4 積分器・同じパラメータで 2501 サンプルを回すため、差は
丸め誤差のレベルに収まるはずです。$\varepsilon > 10^{-9}$ のチャネルが1つでも
あればスクリプトは非ゼロ終了するので、C++ ツールチェーンが使える環境では CI の
回帰検査としてそのまま利用できます。

> この3つ目の検査には C++ コンパイラが必要です。移植作業を行った環境には CMake は
> あってもコンパイラがなかったため、**実行していません**。したがってこれは
> 「実行すれば確認できる」検査であって「ここで確認済み」ではありません。1 と 2 は
> 実行済みで、いずれも通過しています。

---

## 既知の差異

意図的な差異が1点だけあります。C++ の
`DynamicBicycleModel::syncTiresFromParams()` はコーナリングスティフネスは伝播
させますが `params.friction` は伝播させないため、低摩擦プリセットから作った単輪
モデルでもタイヤ側は $\mu = 1.0$ のままになります。Python 移植は既定でこの挙動を
再現しつつ、オプトインの `sync_tires_from_params(sync_friction=True)` を用意して
います。GUI と `performance.py` はこのオプトインを使うため、画面上の全モデルが同じ
路面を共有します。詳細は
[python-api.md](python-api.md#摩擦係数の伝播ギャップ) を参照してください。

これ以外に両実装間で差異があれば、それは移植のバグです。

---

## 数値上の注意

- 両実装とも IEEE-754 倍精度を使い、微分関数の演算順序も揃えてあります。丸め誤差
  レベルの一致は偶然ではなく想定どおりの結果です。
- マヌーバランナは積分*前*に出力をサンプリングします。したがって時刻 $t$ の
  ログは $t + \Delta t$ ではなく $t$ の状態です。C++ の `step_steer` も同じです。
- `simulate()` は指定した継続時間ぴったりで終わるよう最終ステップを短縮します
  （$N = \lfloor T/\Delta t \rfloor$ 回のフルステップの後、最後は
  $T - N\Delta t$）。これも C++ と同じです。
- RK4 の中間段 $k_1 \dots k_4$ は両実装とも `normalize_state()` を通しません。
  正規化されるのは各ステップの最終状態だけです。
